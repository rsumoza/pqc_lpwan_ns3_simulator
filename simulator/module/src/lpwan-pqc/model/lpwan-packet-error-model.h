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
 * \file lpwan-packet-error-model.h
 *
 * \brief LoRa-like PHY/channel packet error model.
 *
 * This model provides:
 *
 *   p_phy = PER(snrDb, sf, payloadBytes)
 *
 * representing PHY/channel decoding failures only.
 *
 * IMPORTANT:
 *   This model does NOT include collisions.
 *   Collision-related failures must be modeled separately and combined at
 *   application level.
 */

#ifndef NS3_LPWAN_PACKET_ERROR_MODEL_H
#define NS3_LPWAN_PACKET_ERROR_MODEL_H

#include "ns3/object.h"
#include "ns3/type-id.h"

#include <cstdint>

namespace ns3
{

/**
 * \ingroup lpwan-pqc
 * \brief Theoretical LoRa-like PHY packet error model.
 *
 * Semantics:
 *   - snrDb: link SNR in dB
 *   - sf: spreading factor
 *   - payloadBytes: on-air payload length in bytes
 *
 * Return value:
 *   p_phy = P(packet decoding failure | snrDb, sf, payloadBytes)
 *
 * with:
 *   0 <= p_phy <= 1
 *
 * Current implementation:
 *   - SF-dependent SNR threshold
 *   - optional SNR calibration offset
 *   - logistic transition around the effective threshold
 */
class LpwanPacketErrorModel : public Object
{
public:
  static TypeId GetTypeId();
  LpwanPacketErrorModel();

  /**
   * \brief Return PHY/channel packet error rate for a given SNR and SF.
   *
   * This function models decoding failures caused by radio conditions only.
   * It must not include collision effects.
   *
   * \param snrDb Link SNR in dB.
   * \param sf Spreading factor in [7,12].
   * \param payloadBytes On-air payload bytes.
   * \return Packet error rate in [0,1].
   */
  double GetPer(double snrDb, uint8_t sf, uint32_t payloadBytes) const;

private:
  /**
   * \brief Calibration offset added to the input SNR before evaluating PER.
   */
  double m_snrOffsetDb{0.0};

  /**
   * \brief Logistic transition slope in dB.
   *
   * Must remain strictly positive.
   */
  double m_slopeDb{1.5};
};

} // namespace ns3

#endif // NS3_LPWAN_PACKET_ERROR_MODEL_H
