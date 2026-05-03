/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * PQC LoRaWAN Exchange Simulator for ns-3
 *
 * Copyright (C) 2026
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * \file lpwan-lora-airtime.h
 *
 * \brief Stateless LoRa airtime and fragmentation utilities.
 *
 * This utility class provides:
 *
 *   - symbol time Tsym(SF,BW),
 *   - Semtech-style time-on-air for explicit header and CRC enabled,
 *   - conservative PHY payload caps used by this simulator,
 *   - payload fragmentation helpers.
 *
 * IMPORTANT:
 *   These utilities are purely PHY/time helpers.
 *   They do NOT model:
 *     - collisions,
 *     - channel PER,
 *     - duty-cycled RX behavior,
 *     - ACK logic,
 *     - retransmissions.
 */

#ifndef NS3_LPWAN_LORA_AIRTIME_H
#define NS3_LPWAN_LORA_AIRTIME_H

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \ingroup lpwan-pqc
 * \brief LoRa physical-layer airtime and payload sizing helpers.
 *
 * This class is stateless and exposes only static utility methods.
 */
class LpwanLoraAirtime
{
public:
  /**
   * \brief Return LoRa symbol time Tsym in seconds.
   *
   * Formula:
   *
   *   Tsym = 2^SF / BW
   *
   * where BW is interpreted in Hz after internal normalization.
   *
   * \param sf Spreading factor, typically in [7,12].
   * \param bwHz Bandwidth in Hz or kHz.
   * \return Symbol time in seconds.
   */
  static double TsymSeconds(uint8_t sf, uint32_t bwHz);

  /**
   * \brief Return LoRa packet airtime in milliseconds.
   *
   * This function uses a Semtech-style explicit-header formula with:
   *
   *   - explicit header      (IH = 0),
   *   - CRC enabled          (CRC = 1),
   *   - preamble length      = 8 symbols,
   *   - low data rate optimization enabled when SF >= 11 and BW = 125 kHz.
   *
   * Coding-rate input convention:
   *
   *   cr in {1,2,3,4}
   *
   * maps to:
   *
   *   CR = 4/(4+cr) = 4/5, 4/6, 4/7, 4/8
   *
   * \param sf Spreading factor, typically in [7,12].
   * \param payloadBytes On-air PHY payload bytes.
   * \param bwHz Bandwidth in Hz or kHz.
   * \param cr Coding-rate parameter in {1..4}.
   * \return Airtime in milliseconds.
   */
  static double AirtimeMs(uint8_t sf,
                          uint32_t payloadBytes,
                          uint32_t bwHz,
                          uint8_t cr);

  /**
   * \brief Return a conservative maximum useful PHY payload for US915.
   *
   * This payload cap is intentionally conservative and is used by this simulator
   * to decide fragmentation.
   *
   * \param sf Spreading factor, typically in [7,12].
   * \return Maximum useful PHY payload in bytes.
   */
  static uint32_t MaxUsefulPayloadUs915(uint8_t sf);

  /**
   * \brief Backward-compatible alias for the simulator payload cap.
   *
   * \param sf Spreading factor, typically in [7,12].
   * \return Maximum payload in bytes.
   */
  static uint32_t MaxPayloadBytes(uint8_t sf)
  {
    return MaxUsefulPayloadUs915(sf);
  }

  /**
   * \brief Split totalBytes into fragments of size <= maxPayloadBytes.
   *
   * If maxPayloadBytes is zero, the implementation defensively uses a minimum
   * effective cap of 1 byte.
   *
   * \param totalBytes Total bytes to split.
   * \param maxPayloadBytes Maximum payload per fragment in bytes.
   * \return Vector of fragment sizes in bytes.
   */
  static std::vector<uint32_t> FragmentSizes(uint32_t totalBytes,
                                             uint32_t maxPayloadBytes);
};

} // namespace ns3

#endif // NS3_LPWAN_LORA_AIRTIME_H
