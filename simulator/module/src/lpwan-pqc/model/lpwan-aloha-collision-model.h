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
 */

#ifndef NS3_LPWAN_ALOHA_COLLISION_MODEL_H
#define NS3_LPWAN_ALOHA_COLLISION_MODEL_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"
#include "ns3/traced-value.h"
#include "ns3/type-id.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup lpwan-pqc
 * \brief ALOHA-style collision model with optional bursty ON/OFF gating
 *        and optional explicit event-driven interference schedule.
 *
 * Canonical responsibility:
 *   - provide lambda_eff(t)
 *   - provide is_on(t)
 *   - provide p_col(t, T)
 *
 * The application should treat this as the single owner of collision
 * probability semantics.
 */
class LpwanAlohaCollisionModel : public Object
{
public:
  struct InterferenceEvent
  {
    double startS = 0.0;
    double endS = 0.0;
    uint32_t channel = 0;
  };

  static TypeId GetTypeId();

  LpwanAlohaCollisionModel();

  /**
   * \brief Return collision probability at time \p nowS for a transmission
   *        of duration \p airtimeMs.
   *
   * If UseExplicitEventCollisions=true, this returns the overlap fraction
   * induced by the explicit interference schedule.
   *
   * Otherwise it returns the analytic Pure ALOHA probability:
   *
   *   p_col = 1 - exp(-2 * lambda_ch * T)
   *
   * where lambda_ch is the average effective rate per logical channel over
   * the vulnerable window.
   */
  double GetCollisionProbabilityAt(double nowS,
                                   double airtimeMs,
                                   uint32_t logicalChannels) const;

  /**
   * \brief Return effective instantaneous total interferer rate lambda_total(t).
   */
  double GetEffectiveLambdaTotal(double nowS) const;

  /**
   * \brief Return bursty ON/OFF state at time \p nowS.
   *
   * Returns true when burstiness is disabled.
   */
  bool GetIsOnAt(double nowS) const;

  /**
   * \brief Return explicit overlap fraction in [0,1] for a transmission window.
   *
   * This is only meaningful when the explicit event schedule has been built.
   * It is also used internally as the explicit collision probability.
   */
  double GetOverlapFractionAt(double nowS,
                              double airtimeMs,
                              uint32_t txChannel) const;

  /**
   * \brief Build explicit interference schedule up to horizon \p horizonS.
   */
  void BuildEventSchedule(double horizonS) const;

  void SetUseExplicitEventCollisions(bool enable);
  bool GetUseExplicitEventCollisions() const;

  uint32_t GetLogicalChannels() const;

private:
  void ValidateConfiguration() const;
  void EnsureInitialized() const;
  void RecomputeLambdaOn() const;
  double SampleOnDurationS() const;
  double SampleOffDurationS() const;
  void UpdateToTime(double nowS) const;
  bool ComputeSquareWaveIsOn(double nowS) const;
  double GetAverageLambdaTotalOverWindow(double centerS,
                                         double halfWindowS) const;
  mutable double m_lastUpdateQueryS{-1.0};
private:
  // --------------------------------------------------------------------------
  // Configurable attributes
  // --------------------------------------------------------------------------
  double m_lambdaTotalAvg{0.0};
  uint32_t m_logicalChannels{64};

  bool m_burstyEnabled{false};
  std::string m_burstyModel{"onoff-exp"};

  double m_onMeanMs{200.0};
  double m_offMeanMs{800.0};
  double m_phaseOffsetMs{0.0};

  bool m_useExplicitEventCollisions{false};
  double m_interfererAirtimeMs{200.0};

  // --------------------------------------------------------------------------
  // Derived / mutable runtime state
  // --------------------------------------------------------------------------
  mutable bool m_initialized{false};

  mutable double m_lambdaOnTotal{0.0};

  // Stateful ON/OFF process for onoff-exp
  mutable bool m_isOn{true};
  mutable double m_nextFlipS{0.0};

  mutable Ptr<ExponentialRandomVariable> m_onRv;
  mutable Ptr<ExponentialRandomVariable> m_offRv;
  mutable Ptr<ExponentialRandomVariable> m_arrivalRv;
  mutable Ptr<UniformRandomVariable> m_channelRv;

  // Explicit event-driven interference schedule
  mutable bool m_scheduleBuilt{false};
  mutable std::vector<InterferenceEvent> m_events;

  // --------------------------------------------------------------------------
  // Traces
  // --------------------------------------------------------------------------
  mutable TracedValue<double> m_tracedEffectiveLambdaTotal;
  mutable TracedValue<bool> m_tracedIsOn;
};

} // namespace ns3

#endif // NS3_LPWAN_ALOHA_COLLISION_MODEL_H
