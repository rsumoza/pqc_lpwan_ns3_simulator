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
 * \file lpwan-pqc-helper.cc
 *
 * \brief Implementation of LpwanPqcHelper.
 */

#include "ns3/lpwan-pqc-helper.h"

#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/ptr.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LpwanPqcHelper");

LpwanPqcHelper::LpwanPqcHelper()
{
  m_appFactory.SetTypeId(PqcExchangeApplication::GetTypeId());
  m_colFactory.SetTypeId(LpwanAlohaCollisionModel::GetTypeId());
  m_perFactory.SetTypeId(LpwanPacketErrorModel::GetTypeId());
}

void
LpwanPqcHelper::SetAppAttribute(const std::string& name, const AttributeValue& value)
{
  m_appFactory.Set(name, value);
}

void
LpwanPqcHelper::SetCollisionAttribute(const std::string& name, const AttributeValue& value)
{
  m_colFactory.Set(name, value);
}

void
LpwanPqcHelper::SetPerAttribute(const std::string& name, const AttributeValue& value)
{
  m_perFactory.Set(name, value);
}

ApplicationContainer
LpwanPqcHelper::Install(const NodeContainer& nodes) const
{
  ApplicationContainer apps;

  m_lastCol = m_colFactory.Create<LpwanAlohaCollisionModel>();
  m_lastPer = m_perFactory.Create<LpwanPacketErrorModel>();

  for (auto it = nodes.Begin(); it != nodes.End(); ++it)
    {
      Ptr<Node> node = *it;
      Ptr<PqcExchangeApplication> app =
          m_appFactory.Create<PqcExchangeApplication>();

      app->SetCollisionModel(m_lastCol);
      app->SetPacketErrorModel(m_lastPer);

      node->AddApplication(app);
      apps.Add(app);
      
    }

  return apps;
}

} // namespace ns3
