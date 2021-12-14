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

#ifndef __PCIB_HOTPLUG_H__
#define __PCIB_HOTPLUG_H__

#define PCIB_HP_LOCK(sc)	mtx_lock((sc)->pcie_hp_lock)
#define PCIB_HP_UNLOCK(sc)	mtx_unlock((sc)->pcie_hp_lock)
#define PCIB_HP_LOCK_ASSERT(sc)	mtx_assert((sc)->pcie_hp_lock, MA_OWNED)

void	pcib_probe_hotplug(struct pcib_softc *sc);
void	pcib_setup_hotplug(struct pcib_softc *sc);
void	pcib_pcie_intr_hotplug(struct pcib_softc *sc);
int	pcib_detach_hotplug(struct pcib_softc *sc);
int	pcib_hotplug_present(struct pcib_softc *sc);
int	pcib_present(struct pcib_softc *sc);

#endif/*__PCIB_HOTPLUG_H__*/
