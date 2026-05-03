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
 * \file lpwan-packet-error-model.cc
 *
 * \brief Implementation of LpwanPacketErrorModel.
 *
 * Current PHY/channel PER model:
 *
 *   x   = (snrDb + snrOffsetDb) - snrThresholdDb(sf)
 *   PER = 1 / (1 + exp(x / slopeDb))
 *
 * Therefore:
 *   - PER -> 1 when x << 0
 *   - PER -> 0 when x >> 0
 *
 * This model represents PHY/channel decoding failures only.
 * It must not include collisions.
 */

#include "ns3/lpwan-packet-error-model.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LpwanPacketErrorModel");
NS_OBJECT_ENSURE_REGISTERED(LpwanPacketErrorModel);

namespace
{

/**
 * \brief Approximate LoRa SNR thresholds (dB) for BW=125 kHz.
 *
 * These values act as soft-threshold anchors for the logistic PER model.
 */
double
SnrThresholdDb(uint8_t sf)
{
  switch (sf)
    {
    case 7:
      return -7.5;
    case 8:
      return -10.0;
    case 9:
      return -12.5;
    case 10:
      return -15.0;
    case 11:
      return -17.5;
    case 12:
      return -20.0;
    default:
      NS_LOG_WARN("SNR_THRESHOLD_DB"
                  << " unsupportedSf=" << static_cast<uint32_t>(sf)
                  << " fallbackSf=7");
      return -7.5;
    }
}

} // namespace

TypeId
LpwanPacketErrorModel::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::LpwanPacketErrorModel")
          .SetParent<Object>()
          .SetGroupName("LpwanPqc")
          .AddConstructor<LpwanPacketErrorModel>()

          .AddAttribute("SnrOffsetDb",
                        "Calibration offset added to the input SNR in dB.",
                        DoubleValue(0.0),
                        MakeDoubleAccessor(&LpwanPacketErrorModel::m_snrOffsetDb),
                        MakeDoubleChecker<double>())

          .AddAttribute("SlopeDb",
                        "Positive logistic slope parameter in dB. "
                        "Smaller values produce a sharper PER transition.",
                        DoubleValue(1.5),
                        MakeDoubleAccessor(&LpwanPacketErrorModel::m_slopeDb),
                        MakeDoubleChecker<double>(0.05));

  return tid;
}

LpwanPacketErrorModel::LpwanPacketErrorModel() = default;

double
LpwanPacketErrorModel::GetPer(double snrDb,
                              uint8_t sf,
                              uint32_t payloadBytes) const
{
  // Preserved in the public contract for interface stability and potential
  // future payload-length sensitivity.
  (void) payloadBytes;

  NS_ASSERT(m_slopeDb > 0.0);

  const double thresholdDb = SnrThresholdDb(sf);
  const double effectiveSnrDb = snrDb + m_snrOffsetDb;
  const double x = effectiveSnrDb - thresholdDb;

  const double per =
      1.0 / (1.0 + std::exp(x / m_slopeDb));

  const double clampedPer = std::clamp(per, 0.0, 1.0);

  return clampedPer;
}

} // namespace ns3
