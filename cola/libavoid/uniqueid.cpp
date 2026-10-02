/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2009-2014  Monash University
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * See the file LICENSE.LGPL distributed with the library.
 *
 * Licensees holding a valid commercial license may use this file in
 * accordance with the commercial license agreement provided with the
 * library.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author(s):  Vladyslav Hnatiuk
*/

#include <atomic>

#include "uniqueid.h"

// Shared by every Router in the process, so it must be safe to call from
// several threads at once: a racing plain increment could hand two objects the
// same id, and a shape's pin set (ordered by id) would then silently drop a
// pin. Ids are only ever compared as creation order between objects of one
// route, so interleaving with other threads' routes does not matter.
static std::atomic<unsigned int> counter{0};

unsigned int getNewUniqueId() {
    unsigned int current = counter.load(std::memory_order_relaxed);
    unsigned int next;
    do {
        // avoid overflow
        next = (current == std::numeric_limits<unsigned int>::max() - 1) ? 1 : current + 1;
    } while (!counter.compare_exchange_weak(current, next, std::memory_order_relaxed));
    return next;
}
