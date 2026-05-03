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
 * \file lpwan-pqc-helper.h
 *
 * \brief Helper for installing the lpwan-pqc application and wiring its models.
 *
 * This helper creates and wires:
 *   - PqcExchangeApplication
 *   - LpwanAlohaCollisionModel
 *   - LpwanPacketErrorModel
 *
 * Contract:
 *   - the scratch driver is the single source of truth for scenario parameters
 *   - attributes are pushed into factories
 *   - one Install() call creates one collision model and one PER model,
 *     shared by all applications created in that call
 */

#ifndef NS3_LPWAN_PQC_HELPER_H
#define NS3_LPWAN_PQC_HELPER_H

#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/ptr.h"

#include "ns3/lpwan-aloha-collision-model.h"
#include "ns3/lpwan-packet-error-model.h"
#include "ns3/pqc-exchange-application.h"

#include <string>

namespace ns3
{

/**
 * \ingroup lpwan-pqc
 * \brief Installer helper for lpwan-pqc components.
 */
class LpwanPqcHelper
{
public:
  LpwanPqcHelper();

  /**
   * \brief Set an Attribute on the application factory.
   */
  void SetAppAttribute(const std::string& name, const AttributeValue& value);

  /**
   * \brief Set an Attribute on the collision model factory.
   */
  void SetCollisionAttribute(const std::string& name, const AttributeValue& value);

  /**
   * \brief Set an Attribute on the PER model factory.
   */
  void SetPerAttribute(const std::string& name, const AttributeValue& value);

  /**
   * \brief Return the last collision model created by Install().
   */
  Ptr<LpwanAlohaCollisionModel> GetCollisionModel() const
  {
    return m_lastCol;
  }

  /**
   * \brief Return the last PER model created by Install().
   */
  Ptr<LpwanPacketErrorModel> GetPerModel() const
  {
    return m_lastPer;
  }

  /**
   * \brief Install applications on all nodes and wire the helper-created models.
   *
   * One collision model instance and one PER model instance are created for this
   * Install() call and shared by all created applications in that call.
   *
   * \param nodes Nodes where the applications will be installed.
   * \return Container with all created applications.
   */
  ApplicationContainer Install(const NodeContainer& nodes) const;

private:
  ObjectFactory m_appFactory;
  ObjectFactory m_colFactory;
  ObjectFactory m_perFactory;

  // Stored for post-Install inspection/trace hookup from scratch.
  // Mutable because Install() is logically const.
  mutable Ptr<LpwanAlohaCollisionModel> m_lastCol;
  mutable Ptr<LpwanPacketErrorModel> m_lastPer;
};

} // namespace ns3

#endif // NS3_LPWAN_PQC_HELPER_H
