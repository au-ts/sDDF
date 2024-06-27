/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

void cache_clean_and_invalidate(unsigned long start, unsigned long end);
void cache_clean(unsigned long start, unsigned long end);
