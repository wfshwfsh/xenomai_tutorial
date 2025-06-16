/*****************************************************************************
 *
 *  Copyright (C) 2006-2023  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT master userspace library.
 *
 *  The IgH EtherCAT master userspace library is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU Lesser General
 *  Public License as published by the Free Software Foundation; version 2.1
 *  of the License.
 *
 *  The IgH EtherCAT master userspace library is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 ****************************************************************************/

#include "include/ecrt.h"

/****************************************************************************/

struct ec_soe_request {
    ec_soe_request_t *next; /**< List header. */
    ec_slave_config_t *config; /**< Parent slave configuration. */
    unsigned int index; /**< Request index (identifier). */
    uint8_t drive_no; /**< SoE drive number. */
    uint16_t idn; /**< SoE ID number. */
    uint8_t *data; /**< Pointer to SoE data. */
    size_t mem_size; /**< Size of SoE data memory. */
    size_t data_size; /**< Size of SoE data. */
};

/****************************************************************************/

void ec_soe_request_clear(ec_soe_request_t *);

/****************************************************************************/
