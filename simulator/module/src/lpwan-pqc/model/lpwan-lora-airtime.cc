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
 * \file lpwan-lora-airtime.cc
 *
 * \brief Implementation of LoRa time-on-air utilities used by lpwan-pqc.
 */

#include "ns3/lpwan-lora-airtime.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LpwanLoraAirtime");

namespace
{

uint32_t
NormalizeBwHz(uint32_t bwHz)
{
  if (bwHz == 0)
    {
      NS_LOG_WARN("NORMALIZE_BW_HZ"
                  << " inputBw=0"
                  << " fallbackBwHz=125000");
      return 125000;
    }

  if (bwHz < 1000)
    {
      const uint32_t normalized = bwHz * 1000;
      return normalized;
    }

  return bwHz;
}

} // namespace

double
LpwanLoraAirtime::TsymSeconds(uint8_t sf, uint32_t bwHz)
{
  const uint32_t bwHzN = NormalizeBwHz(bwHz);
  const double bw = static_cast<double>(bwHzN);

  const double tsymS =
      std::pow(2.0, static_cast<double>(sf)) / bw;

  return tsymS;
}

double
LpwanLoraAirtime::AirtimeMs(uint8_t sf,
                            uint32_t payloadBytes,
                            uint32_t bwHz,
                            uint8_t cr)
{
  const uint32_t bwHzN = NormalizeBwHz(bwHz);
  const double tsymS = TsymSeconds(sf, bwHzN);

  const double nPreamble = 8.0;
  const int ih = 0;
  const int crc = 1;
  const int de = (sf >= 11 && bwHzN == 125000) ? 1 : 0;

  const double sfD = static_cast<double>(sf);
  const double pl = static_cast<double>(payloadBytes);
  const double crD = static_cast<double>(std::clamp<uint8_t>(cr, 1, 4));

  const double numerator =
      8.0 * pl - 4.0 * sfD + 28.0 + 16.0 * crc - 20.0 * ih;

  const double denominator =
      4.0 * (sfD - 2.0 * de);

  double payloadSymbNb = 8.0;

  if (denominator <= 0.0)
    {
      NS_LOG_WARN("LORA_AIRTIME_MS"
                  << " sf=" << static_cast<uint32_t>(sf)
                  << " payloadBytes=" << payloadBytes
                  << " bwInput=" << bwHz
                  << " bwHz=" << bwHzN
                  << " cr=" << static_cast<uint32_t>(cr)
                  << " denominator=" << denominator
                  << " note=invalid_denominator_using_min_payload_symbols");
    }
  else if (numerator > 0.0)
    {
      const double tmp = std::ceil(numerator / denominator);
      payloadSymbNb = 8.0 + std::max(0.0, tmp * (crD + 4.0));
    }

  const double totalSymb =
      nPreamble + 4.25 + payloadSymbNb;

  const double toaS = tsymS * totalSymb;
  const double toaMs = toaS * 1000.0;

  return toaMs;
}

uint32_t
LpwanLoraAirtime::MaxUsefulPayloadUs915(uint8_t sf)
{
  uint32_t maxPayload = 51;

  switch (sf)
    {
    case 7:
      maxPayload = 222;
      break;
    case 8:
      maxPayload = 222;
      break;
    case 9:
      maxPayload = 115;
      break;
    case 10:
      maxPayload = 51;
      break;
    case 11:
      maxPayload = 51;
      break;
    case 12:
      maxPayload = 51;
      break;
    default:
      NS_LOG_WARN("MAX_USEFUL_PAYLOAD_US915"
                  << " unsupportedSf=" << static_cast<uint32_t>(sf)
                  << " fallbackPayload=51");
      maxPayload = 51;
      break;
    }

  return maxPayload;
}

std::vector<uint32_t>
LpwanLoraAirtime::FragmentSizes(uint32_t totalBytes,
                                uint32_t maxPayloadBytes)
{
  std::vector<uint32_t> out;
  const uint32_t cap = std::max<uint32_t>(1, maxPayloadBytes);

  uint32_t remaining = totalBytes;
  while (remaining > 0)
    {
      const uint32_t take = std::min(cap, remaining);
      out.push_back(take);
      remaining -= take;
    }

  return out;
}

} // namespace ns3
