/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

#ifndef DEBUG_PWR_H
#define DEBUG_PWR_H

#include <zephyr/kernel.h>

/**
 * @brief Print detailed power management debug information
 *
 * This function reads and displays power domain status, retention settings,
 * voltage trim settings, and other power-related hardware registers.
 */
void debug_pwr_print(const struct shell *shell);

#endif /* DEBUG_PWR_H */
