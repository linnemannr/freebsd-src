/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 1994,1995 Stefan Esser, Wolfgang StanglMeier
 * Copyright (c) 2000 Michael Smith <msmith@freebsd.org>
 * Copyright (c) 2000 BSDi
 * Copyright (c) 2021 Spectra Logic Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_pci.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pci_private.h>
#include <dev/pci/pcib_private.h>

/*
 * PCI-express HotPlug support.
 */
static int pci_enable_pcie_hp = 1;
SYSCTL_DECL(_hw_pci);
SYSCTL_INT(_hw_pci, OID_AUTO, enable_pcie_hp, CTLFLAG_RDTUN,
    &pci_enable_pcie_hp, 0,
    "Enable support for native PCI-express HotPlug.");

TASKQUEUE_DEFINE_THREAD(pci_hp);

void
pcib_probe_hotplug(struct pcib_softc *sc)
{
	device_t dev;
	uint32_t link_cap;
	uint16_t link_sta, slot_sta;

	if (!pci_enable_pcie_hp)
		return;

	dev = sc->dev;
	if (pci_find_cap(dev, PCIY_EXPRESS, NULL) != 0)
		return;

	if (!(pcie_read_config(dev, PCIER_FLAGS, 2) & PCIEM_FLAGS_SLOT))
		return;

	sc->pcie_slot_cap = pcie_read_config(dev, PCIER_SLOT_CAP, 4);

	if ((sc->pcie_slot_cap & PCIEM_SLOT_CAP_HPC) == 0)
		return;
	link_cap = pcie_read_config(dev, PCIER_LINK_CAP, 4);
	if ((link_cap & PCIEM_LINK_CAP_DL_ACTIVE) == 0)
		return;

	/*
	 * Some devices report that they have an MRL when they actually
	 * do not.  Since they always report that the MRL is open, child
	 * devices would be ignored.  Try to detect these devices and
	 * ignore their claim of HotPlug support.
	 *
	 * If there is an open MRL but the Data Link Layer is active,
	 * the MRL is not real.
	 */
	if ((sc->pcie_slot_cap & PCIEM_SLOT_CAP_MRLSP) != 0) {
		link_sta = pcie_read_config(dev, PCIER_LINK_STA, 2);
		slot_sta = pcie_read_config(dev, PCIER_SLOT_STA, 2);
		if ((slot_sta & PCIEM_SLOT_STA_MRLSS) != 0 &&
		    (link_sta & PCIEM_LINK_STA_DL_ACTIVE) != 0) {
			return;
		}
	}

	/*
	 * Now that we're sure we want to do hot plug, ask the
	 * firmware, if any, if that's OK.
	 */
	if (pcib_request_feature(dev, PCI_FEATURE_HP) != 0) {
		if (bootverbose)
			device_printf(dev, "Unable to activate hot plug feature.\n");
		return;
	}

	sc->hp.flags |= PCIB_HOTPLUG_ENABLED;
}

/*
 * Send a HotPlug command to the slot control register.  If this slot
 * uses command completion interrupts and a previous command is still
 * in progress, then the command is dropped.  Once the previous
 * command completes or times out, pcib_pcie_hotplug_update() will be
 * invoked to post a new command based on the slot's state at that
 * time.
 */
static void
pcib_pcie_hotplug_command(struct pcib_softc *sc, uint16_t val, uint16_t mask)
{
	device_t dev;
	uint16_t ctl, new;

	dev = sc->dev;

	if (sc->hp.flags & PCIB_HOTPLUG_CMD_PENDING)
		return;

	ctl = pcie_read_config(dev, PCIER_SLOT_CTL, 2);
	new = (ctl & ~mask) | val;
	if (new == ctl)
		return;
	if (bootverbose)
		device_printf(dev, "HotPlug command: %04x -> %04x\n", ctl, new);
	pcie_write_config(dev, PCIER_SLOT_CTL, new, 2);
	if (!(sc->pcie_slot_cap & PCIEM_SLOT_CAP_NCCS) &&
	    (ctl & new) & PCIEM_SLOT_CTL_CCIE) {
		sc->hp.flags |= PCIB_HOTPLUG_CMD_PENDING;
		if (!cold)
			taskqueue_enqueue_timeout(taskqueue_pci_hp,
			    &sc->hp.pcie_cc_task, hz);
	}
}

static void
pcib_pcie_hotplug_command_completed(struct pcib_softc *sc)
{
	device_t dev;

	dev = sc->dev;

	if (bootverbose)
		device_printf(dev, "Command Completed\n");
	if (!(sc->hp.flags & PCIB_HOTPLUG_CMD_PENDING))
		return;
	taskqueue_cancel_timeout(taskqueue_pci_hp,
	    &sc->hp.pcie_cc_task, NULL);
	sc->hp.flags &= ~PCIB_HOTPLUG_CMD_PENDING;
	wakeup(sc);
}

/*
 * Returns true if a card is fully inserted from the user's
 * perspective.  It may not yet be ready for access, but the driver
 * can now start enabling access if necessary.
 */
static bool
pcib_hotplug_inserted(struct pcib_softc *sc)
{

	/* Pretend the card isn't present if a detach is forced. */
	if (sc->hp.flags & PCIB_DETACHING)
		return (false);

	/* Card must be present in the slot. */
	if ((sc->pcie_slot_sta & PCIEM_SLOT_STA_PDS) == 0)
		return (false);

	/* A power fault implicitly turns off power to the slot. */
	if (sc->pcie_slot_sta & PCIEM_SLOT_STA_PFD)
		return (false);

	/* If the MRL is disengaged, the slot is powered off. */
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_MRLSP &&
	    (sc->pcie_slot_sta & PCIEM_SLOT_STA_MRLSS) != 0)
		return (false);

	return (true);
}

/*
 * Returns -1 if the card is fully inserted, powered, and ready for
 * access. Otherwise, returns 0.
 */
int
pcib_hotplug_present(struct pcib_softc *sc)
{
	if (!(sc->hp.flags & PCIB_HOTPLUG_ENABLED))
		return 0;

	/* Card must be inserted. */
	if (!pcib_hotplug_inserted(sc))
		return (0);

	/* Require the Data Link Layer to be active. */
	if (!(sc->pcie_link_sta & PCIEM_LINK_STA_DL_ACTIVE))
		return (0);

	return (-1);
}

static void
pcib_pcie_hotplug_update(struct pcib_softc *sc, uint16_t val, uint16_t mask,
    bool schedule_task)
{
	bool card_inserted, ei_engaged;

	/* Clear DETACHING if Presence Detect has cleared. */
	if ((sc->pcie_slot_sta & (PCIEM_SLOT_STA_PDC | PCIEM_SLOT_STA_PDS)) ==
	    PCIEM_SLOT_STA_PDC)
		sc->hp.flags &= ~PCIB_DETACHING;

	card_inserted = pcib_hotplug_inserted(sc);

	/* Turn the power indicator on if a card is inserted. */
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_PIP) {
		mask |= PCIEM_SLOT_CTL_PIC;
		if (card_inserted)
			val |= PCIEM_SLOT_CTL_PI_ON;
		else if (sc->hp.flags & PCIB_DETACH_PENDING)
			val |= PCIEM_SLOT_CTL_PI_BLINK;
		else
			val |= PCIEM_SLOT_CTL_PI_OFF;
	}

	/* Turn the power on via the Power Controller if a card is inserted. */
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_PCP) {
		mask |= PCIEM_SLOT_CTL_PCC;
		if (card_inserted)
			val |= PCIEM_SLOT_CTL_PC_ON;
		else
			val |= PCIEM_SLOT_CTL_PC_OFF;
	}

	/*
	 * If a card is inserted, enable the Electromechanical
	 * Interlock.  If a card is not inserted (or we are in the
	 * process of detaching), disable the Electromechanical
	 * Interlock.
	 */
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_EIP) {
		mask |= PCIEM_SLOT_CTL_EIC;
		ei_engaged = (sc->pcie_slot_sta & PCIEM_SLOT_STA_EIS) != 0;
		if (card_inserted != ei_engaged)
			val |= PCIEM_SLOT_CTL_EIC;
	}

	/*
	 * Start a timer to see if the Data Link Layer times out.
	 * Note that we only start the timer if Presence Detect or MRL Sensor
	 * changed on this interrupt.  Stop any scheduled timer if
	 * the Data Link Layer is active.
	 */
	if (card_inserted &&
	    !(sc->pcie_link_sta & PCIEM_LINK_STA_DL_ACTIVE) &&
	    sc->pcie_slot_sta &
	    (PCIEM_SLOT_STA_MRLSC | PCIEM_SLOT_STA_PDC)) {
		if (cold)
			device_printf(sc->dev,
			    "Data Link Layer inactive\n");
		else
			taskqueue_enqueue_timeout(taskqueue_pci_hp,
			    &sc->hp.pcie_dll_task, hz);
	} else if (sc->pcie_link_sta & PCIEM_LINK_STA_DL_ACTIVE)
		taskqueue_cancel_timeout(taskqueue_pci_hp, &sc->hp.pcie_dll_task,
		    NULL);

	pcib_pcie_hotplug_command(sc, val, mask);

	/*
	 * During attach the child "pci" device is added synchronously;
	 * otherwise, the task is scheduled to manage the child
	 * device.
	 */
	if (schedule_task &&
	    (pcib_hotplug_present(sc) != 0) != (sc->child != NULL))
		taskqueue_enqueue(taskqueue_pci_hp, &sc->hp.pcie_hp_task);
}

void
pcib_pcie_intr_hotplug(struct pcib_softc *sc)
{
	device_t dev;
	uint16_t old_slot_sta;

	if (!(sc->hp.flags & PCIB_HOTPLUG_ENABLED))
		return;

	dev = sc->dev;
	PCIB_HP_LOCK(sc);
	old_slot_sta = sc->pcie_slot_sta;
	sc->pcie_slot_sta = pcie_read_config(dev, PCIER_SLOT_STA, 2);

	/* Clear the events just reported. */
	pcie_write_config(dev, PCIER_SLOT_STA, sc->pcie_slot_sta, 2);

	if (bootverbose)
		device_printf(dev, "HotPlug interrupt: %#x\n",
		    sc->pcie_slot_sta);

	if (sc->pcie_slot_sta & PCIEM_SLOT_STA_ABP) {
		if (sc->hp.flags & PCIB_DETACH_PENDING) {
			device_printf(dev,
			    "Attention Button Pressed: Detach Cancelled\n");
			sc->hp.flags &= ~PCIB_DETACH_PENDING;
			taskqueue_cancel_timeout(taskqueue_pci_hp,
			    &sc->hp.pcie_ab_task, NULL);
		} else if (old_slot_sta & PCIEM_SLOT_STA_PDS) {
			/* Only initiate detach sequence if device present. */
			device_printf(dev,
		    "Attention Button Pressed: Detaching in 5 seconds\n");
			sc->hp.flags |= PCIB_DETACH_PENDING;
			taskqueue_enqueue_timeout(taskqueue_pci_hp,
			    &sc->hp.pcie_ab_task, 5 * hz);
		}
	}
	if (sc->pcie_slot_sta & PCIEM_SLOT_STA_PFD)
		device_printf(dev, "Power Fault Detected\n");
	if (sc->pcie_slot_sta & PCIEM_SLOT_STA_MRLSC)
		device_printf(dev, "MRL Sensor Changed to %s\n",
		    sc->pcie_slot_sta & PCIEM_SLOT_STA_MRLSS ? "open" :
		    "closed");
	if (bootverbose && sc->pcie_slot_sta & PCIEM_SLOT_STA_PDC)
		device_printf(dev, "Presence Detect Changed to %s\n",
		    sc->pcie_slot_sta & PCIEM_SLOT_STA_PDS ? "card present" :
		    "empty");
	if (sc->pcie_slot_sta & PCIEM_SLOT_STA_CC)
		pcib_pcie_hotplug_command_completed(sc);
	if (sc->pcie_slot_sta & PCIEM_SLOT_STA_DLLSC) {
		sc->pcie_link_sta = pcie_read_config(dev, PCIER_LINK_STA, 2);
		if (bootverbose)
			device_printf(dev,
			    "Data Link Layer State Changed to %s\n",
			    sc->pcie_link_sta & PCIEM_LINK_STA_DL_ACTIVE ?
			    "active" : "inactive");
	}

	pcib_pcie_hotplug_update(sc, 0, 0, true);
}

static void
pcib_pcie_hotplug_task(void *context, int pending)
{
	struct pcib_softc *sc;
	device_t dev;

	sc = context;
	PCIB_HP_LOCK(sc);
	dev = sc->dev;
	if (pcib_hotplug_present(sc) != 0) {
		if (sc->child == NULL) {
			sc->child = device_add_child(dev, "pci", -1);
			bus_generic_attach(dev);
		}
	} else {
		if (sc->child != NULL) {
			if (device_delete_child(dev, sc->child) == 0)
				sc->child = NULL;
		}
	}
	PCIB_HP_UNLOCK(sc);
}

static void
pcib_pcie_ab_timeout(void *arg, int pending)
{
	struct pcib_softc *sc = arg;

	PCIB_HP_LOCK(sc);
	if (sc->hp.flags & PCIB_DETACH_PENDING) {
		sc->hp.flags |= PCIB_DETACHING;
		sc->hp.flags &= ~PCIB_DETACH_PENDING;
	}
	PCIB_HP_UNLOCK(sc);
}

static void
pcib_pcie_cc_timeout(void *arg, int pending)
{
	struct pcib_softc *sc = arg;
	device_t dev = sc->dev;
	uint16_t sta;

	PCIB_HP_LOCK(sc);
	sta = pcie_read_config(dev, PCIER_SLOT_STA, 2);
	if (!(sta & PCIEM_SLOT_STA_CC)) {
		device_printf(dev, "HotPlug Command Timed Out\n");
		sc->hp.flags &= ~PCIB_HOTPLUG_CMD_PENDING;
	} else {
		device_printf(dev,
	    "Missed HotPlug interrupt waiting for Command Completion\n");
		pcib_pcie_intr_hotplug(sc);
	}
	PCIB_HP_UNLOCK(sc);
}

static void
pcib_pcie_dll_timeout(void *arg, int pending)
{
	struct pcib_softc *sc = arg;
	device_t dev = sc->dev;
	uint16_t sta;

	PCIB_HP_LOCK(sc);
	sta = pcie_read_config(dev, PCIER_LINK_STA, 2);
	if (!(sta & PCIEM_LINK_STA_DL_ACTIVE)) {
		device_printf(dev,
		    "Timed out waiting for Data Link Layer Active\n");
		sc->hp.flags |= PCIB_DETACHING;
		pcib_pcie_hotplug_update(sc, 0, 0, true);
	} else if (sta != sc->pcie_link_sta) {
		device_printf(dev,
		    "Missed HotPlug interrupt waiting for DLL Active\n");
		pcib_pcie_intr_hotplug(sc);
	}
}

void
pcib_setup_hotplug(struct pcib_softc *sc)
{
	device_t dev;
	uint16_t mask, val;

	if (!(sc->hp.flags & PCIB_HOTPLUG_ENABLED))
		return;

	dev = sc->dev;
	TASK_INIT(&sc->hp.pcie_hp_task, 0, pcib_pcie_hotplug_task, sc);
	TIMEOUT_TASK_INIT(taskqueue_pci_hp, &sc->hp.pcie_ab_task, 0,
	    pcib_pcie_ab_timeout, sc);
	TIMEOUT_TASK_INIT(taskqueue_pci_hp, &sc->hp.pcie_cc_task, 0,
	    pcib_pcie_cc_timeout, sc);
	TIMEOUT_TASK_INIT(taskqueue_pci_hp, &sc->hp.pcie_dll_task, 0,
	    pcib_pcie_dll_timeout, sc);
	sc->hp.pcie_hp_lock = &Giant;
	sc->pcie_link_sta = pcie_read_config(dev, PCIER_LINK_STA, 2);
	sc->pcie_slot_sta = pcie_read_config(dev, PCIER_SLOT_STA, 2);

	/* Clear any events previously pending. */
	pcie_write_config(dev, PCIER_SLOT_STA, sc->pcie_slot_sta, 2);

	/* Enable HotPlug events. */
	mask = PCIEM_SLOT_CTL_DLLSCE | PCIEM_SLOT_CTL_HPIE |
	    PCIEM_SLOT_CTL_CCIE | PCIEM_SLOT_CTL_PDCE | PCIEM_SLOT_CTL_MRLSCE |
	    PCIEM_SLOT_CTL_PFDE | PCIEM_SLOT_CTL_ABPE;
	val = PCIEM_SLOT_CTL_DLLSCE | PCIEM_SLOT_CTL_HPIE | PCIEM_SLOT_CTL_PDCE;
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_APB)
		val |= PCIEM_SLOT_CTL_ABPE;
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_PCP)
		val |= PCIEM_SLOT_CTL_PFDE;
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_MRLSP)
		val |= PCIEM_SLOT_CTL_MRLSCE;
	if (!(sc->pcie_slot_cap & PCIEM_SLOT_CAP_NCCS))
		val |= PCIEM_SLOT_CTL_CCIE;

	/* Turn the attention indicator off. */
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_AIP) {
		mask |= PCIEM_SLOT_CTL_AIC;
		val |= PCIEM_SLOT_CTL_AI_OFF;
	}

	pcib_pcie_hotplug_update(sc, val, mask, false);
}

int
pcib_detach_hotplug(struct pcib_softc *sc)
{
	uint16_t mask, val;

	if (!(sc->hp.flags & PCIB_HOTPLUG_ENABLED))
		return 0;

	/* Disable the card in the slot and force it to detach. */
	if (sc->hp.flags & PCIB_DETACH_PENDING) {
		sc->hp.flags &= ~PCIB_DETACH_PENDING;
		taskqueue_cancel_timeout(taskqueue_pci_hp, &sc->hp.pcie_ab_task,
		    NULL);
	}
	sc->hp.flags |= PCIB_DETACHING;

	if (sc->hp.flags & PCIB_HOTPLUG_CMD_PENDING) {
		taskqueue_cancel_timeout(taskqueue_pci_hp, &sc->hp.pcie_cc_task,
		    NULL);
		tsleep(sc, 0, "hpcmd", hz);
		sc->hp.flags &= ~PCIB_HOTPLUG_CMD_PENDING;
	}

	/* Disable HotPlug events. */
	mask = PCIEM_SLOT_CTL_DLLSCE | PCIEM_SLOT_CTL_HPIE |
	    PCIEM_SLOT_CTL_CCIE | PCIEM_SLOT_CTL_PDCE | PCIEM_SLOT_CTL_MRLSCE |
	    PCIEM_SLOT_CTL_PFDE | PCIEM_SLOT_CTL_ABPE;
	val = 0;

	/* Turn the attention indicator off. */
	if (sc->pcie_slot_cap & PCIEM_SLOT_CAP_AIP) {
		mask |= PCIEM_SLOT_CTL_AIC;
		val |= PCIEM_SLOT_CTL_AI_OFF;
	}

	pcib_pcie_hotplug_update(sc, val, mask, false);

	taskqueue_drain(taskqueue_pci_hp, &sc->hp.pcie_hp_task);
	taskqueue_drain_timeout(taskqueue_pci_hp, &sc->hp.pcie_ab_task);
	taskqueue_drain_timeout(taskqueue_pci_hp, &sc->hp.pcie_cc_task);
	taskqueue_drain_timeout(taskqueue_pci_hp, &sc->hp.pcie_dll_task);
	return (0);
}
