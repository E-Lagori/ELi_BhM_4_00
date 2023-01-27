/*
 * Redirects the header to appropriate version of microcontroler header file 
 * for the processor module that is being used
 *
 * Copyright (C) 2018  E-Lagori https://github.com/E-Lagori/ELi_BhM_4_00
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This file contains the code for ELi_BhM_4_00 library.
 *
 */

#ifndef ELi_BhM_4_00_H
#define ELi_BhM_4_00_H

#include <ELi_McM_4_00.h>

#ifdef ESP32
  #include "esp32/BhM_ESP32D_4_00.h"
#else 
  #error "Controller not supported"
#endif
#endif
