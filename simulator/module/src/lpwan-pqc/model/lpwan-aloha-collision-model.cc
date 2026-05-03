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

#include "ns3/lpwan-aloha-collision-model.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LpwanAlohaCollisionModel");
NS_OBJECT_ENSURE_REGISTERED(LpwanAlohaCollisionModel);

TypeId
LpwanAlohaCollisionModel::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::LpwanAlohaCollisionModel")
          .SetParent<Object>()
          .SetGroupName("LpwanPqc")
          .AddConstructor<LpwanAlohaCollisionModel>()

          .AddAttribute("LambdaTotalAvg",
                        "Long-run average total interferer attempt rate "
                        "lambda_total [attempts/s].",
                        DoubleValue(0.0),
                        MakeDoubleAccessor(&LpwanAlohaCollisionModel::m_lambdaTotalAvg),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("LogicalChannels",
                        "Default number of logical channels M used when callers "
                        "pass logicalChannels=0.",
                        UintegerValue(64),
                        MakeUintegerAccessor(&LpwanAlohaCollisionModel::m_logicalChannels),
                        MakeUintegerChecker<uint32_t>(1))

          .AddAttribute("BurstyEnabled",
                        "Enable ON/OFF bursty gating of interferers.",
                        BooleanValue(false),
                        MakeBooleanAccessor(&LpwanAlohaCollisionModel::m_burstyEnabled),
                        MakeBooleanChecker())

          .AddAttribute("BurstyModel",
                        "Bursty model name. Supported values: "
                        "onoff-exp, onoff-square.",
                        StringValue("onoff-exp"),
                        MakeStringAccessor(&LpwanAlohaCollisionModel::m_burstyModel),
                        MakeStringChecker())

          .AddAttribute("OnMeanMs",
                        "ON duration parameter in milliseconds. "
                        "For onoff-square this is the deterministic ON duration. "
                        "For onoff-exp this is the exponential mean.",
                        DoubleValue(200.0),
                        MakeDoubleAccessor(&LpwanAlohaCollisionModel::m_onMeanMs),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("OffMeanMs",
                        "OFF duration parameter in milliseconds. "
                        "For onoff-square this is the deterministic OFF duration. "
                        "For onoff-exp this is the exponential mean.",
                        DoubleValue(800.0),
                        MakeDoubleAccessor(&LpwanAlohaCollisionModel::m_offMeanMs),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("PhaseOffsetMs",
                        "Phase offset in milliseconds for the deterministic "
                        "onoff-square model. Ignored for other bursty models.",
                        DoubleValue(0.0),
                        MakeDoubleAccessor(&LpwanAlohaCollisionModel::m_phaseOffsetMs),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("UseExplicitEventCollisions",
                        "If true, collision probability is obtained from the explicit "
                        "event-driven interference schedule instead of the analytic "
                        "Pure ALOHA approximation.",
                        BooleanValue(false),
                        MakeBooleanAccessor(&LpwanAlohaCollisionModel::m_useExplicitEventCollisions),
                        MakeBooleanChecker())

          .AddAttribute("InterfererAirtimeMs",
                        "Airtime in milliseconds used to instantiate explicit "
                        "interference events. Used only when "
                        "UseExplicitEventCollisions=true.",
                        DoubleValue(200.0),
                        MakeDoubleAccessor(&LpwanAlohaCollisionModel::m_interfererAirtimeMs),
                        MakeDoubleChecker<double>(0.0))

          .AddTraceSource("EffectiveLambdaTotal",
                          "Effective instantaneous lambda_total(t) after bursty "
                          "ON/OFF gating [attempts/s].",
                          MakeTraceSourceAccessor(&LpwanAlohaCollisionModel::m_tracedEffectiveLambdaTotal),
                          "ns3::TracedValueCallback::Double")

          .AddTraceSource("IsOn",
                          "Instantaneous ON/OFF bursty state (true=ON). "
                          "Always true when burstiness is disabled.",
                          MakeTraceSourceAccessor(&LpwanAlohaCollisionModel::m_tracedIsOn),
                          "ns3::TracedValueCallback::Bool");

  return tid;
}

LpwanAlohaCollisionModel::LpwanAlohaCollisionModel() = default;

uint32_t
LpwanAlohaCollisionModel::GetLogicalChannels() const
{
  return m_logicalChannels;
}

void
LpwanAlohaCollisionModel::EnsureInitialized() const
{
  /*
   * Contract:
   *   Initialize the collision model exactly once.
   *
   * Responsibilities:
   *   - validate public configuration
   *   - create RNG objects
   *   - compute derived ON-state load
   *   - initialize bursty-state evolution variables
   *   - initialize trace-visible state
   */

/*
   * Validate configuration on every entry.
   *
   * Rationale:
   *   ns-3 attributes may change after initialization. We must guarantee
   *   the model is always in a valid state before use.
   */
  ValidateConfiguration();

  if (m_initialized)
    {
      return;
    }

  m_onRv = CreateObject<ExponentialRandomVariable>();
  m_offRv = CreateObject<ExponentialRandomVariable>();
  m_arrivalRv = CreateObject<ExponentialRandomVariable>();
  m_channelRv = CreateObject<UniformRandomVariable>();

  RecomputeLambdaOn();

  /*
   * Reset temporal query bookkeeping for stateful bursty updates.
   */
  m_lastUpdateQueryS = -1.0;

  /*
   * Initial bursty state:
   *
   * - stateful bursty models start in ON state and schedule the first flip
   *   after one sampled ON duration
   *
   * - deterministic square-wave models do not rely on m_nextFlipS for state
   *   evolution, but we still initialize it defensively
   */
  m_isOn = true;
  m_nextFlipS = SampleOnDurationS();
  if (m_nextFlipS <= 0.0)
    {
      m_nextFlipS = 1e-9;
    }

  m_initialized = true;

  /*
   * Initialize trace-visible state consistently with the configured model.
   */
  if (!m_burstyEnabled)
    {
      m_tracedIsOn = true;
      m_tracedEffectiveLambdaTotal = m_lambdaTotalAvg;
    }
  else if (m_burstyModel == "onoff-square")
    {
      const bool isOn0 = ComputeSquareWaveIsOn(0.0);
      m_tracedIsOn = isOn0;
      m_tracedEffectiveLambdaTotal = isOn0 ? m_lambdaOnTotal : 0.0;
    }
  else
    {
      m_tracedIsOn = m_isOn;
      m_tracedEffectiveLambdaTotal = m_isOn ? m_lambdaOnTotal : 0.0;
    }
}

void
LpwanAlohaCollisionModel::RecomputeLambdaOn() const
{
  /*
   * Contract:
   *   Recompute the ON-state total attempt rate m_lambdaOnTotal so that the
   *   long-run average attempt rate remains equal to m_lambdaTotalAvg under
   *   bursty ON/OFF operation.
   *
   * Average-preserving relation:
   *
   *   lambda_avg = duty * lambda_on
   *
   * therefore:
   *
   *   lambda_on = lambda_avg / duty
   *
   * where:
   *
   *   duty = on_time / (on_time + off_time)
   *
   * Important:
   *   - m_lambdaOnTotal is a derived internal quantity, not a directly
   *     configured public parameter.
   *   - This function enforces consistency between the configured long-run
   *     average load and the instantaneous ON-state load.
   */

  const double onS = std::max(0.0, m_onMeanMs) / 1000.0;
  const double offS = std::max(0.0, m_offMeanMs) / 1000.0;

  const double periodS = onS + offS;
  const double duty = (periodS > 0.0) ? (onS / periodS) : 1.0;

  /*
   * Degenerate fallback:
   * if the total ON/OFF period collapses to zero, preserve the configured
   * average rate directly.
   *
   * In normal operation, ValidateConfiguration() should reject invalid bursty
   * settings when they matter semantically.
   */
  if (periodS <= 0.0)
    {
      m_lambdaOnTotal = std::max(0.0, m_lambdaTotalAvg);
      return;
    }

  /*
   * A strictly zero ON duty cycle cannot sustain a positive average load.
   */
  NS_ABORT_MSG_IF(duty <= 0.0 && m_lambdaTotalAvg > 0.0,
                  "Invalid bursty configuration: ON duty cycle is zero while "
                  "LambdaTotalAvg is positive.");

  if (duty > 0.0)
    {
      m_lambdaOnTotal = m_lambdaTotalAvg / duty;
    }
  else
    {
      m_lambdaOnTotal = 0.0;
    }

  m_lambdaOnTotal = std::max(0.0, m_lambdaOnTotal);
}

double
LpwanAlohaCollisionModel::SampleOnDurationS() const
{
  /*
   * Contract:
   *   Sample one ON-state duration in seconds.
   *
   * Behavior:
   *   - if the configured ON mean is <= 0, return 0 as a degenerate duration
   *   - otherwise return a strictly positive sampled duration
   *
   * Important:
   *   The strictly positive lower bound avoids degenerate zero-length state
   *   transitions that could destabilize stateful bursty updates.
   */

  NS_ASSERT(m_onRv != nullptr);

  const double meanS = std::max(0.0, m_onMeanMs) / 1000.0;
  if (meanS <= 0.0)
    {
      return 0.0;
    }

  m_onRv->SetAttribute("Mean", DoubleValue(meanS));

  const double sampleS = m_onRv->GetValue();
  return std::max(sampleS, 1e-9);
}

double
LpwanAlohaCollisionModel::SampleOffDurationS() const
{
  /*
   * Contract:
   *   Sample one OFF-state duration in seconds.
   *
   * Behavior:
   *   - if the configured OFF mean is <= 0, return 0 as a degenerate duration
   *   - otherwise return a strictly positive sampled duration
   *
   * Important:
   *   The strictly positive lower bound avoids degenerate zero-length state
   *   transitions that could destabilize stateful bursty updates.
   */

  NS_ASSERT(m_offRv != nullptr);

  const double meanS = std::max(0.0, m_offMeanMs) / 1000.0;
  if (meanS <= 0.0)
    {
      return 0.0;
    }

  m_offRv->SetAttribute("Mean", DoubleValue(meanS));

  const double sampleS = m_offRv->GetValue();
  return std::max(sampleS, 1e-9);
}

bool
LpwanAlohaCollisionModel::ComputeSquareWaveIsOn(double nowS) const
{
  /*
   * Contract:
   *   Return whether the deterministic square-wave bursty process is ON at
   *   time nowS.
   *
   * Timing model:
   *   - ON duration  = m_onMeanMs
   *   - OFF duration = m_offMeanMs
   *   - phase offset = m_phaseOffsetMs
   *
   * Square-wave definition:
   *   Let:
   *
   *     onS        = m_onMeanMs  / 1000
   *     offS       = m_offMeanMs / 1000
   *     periodS    = onS + offS
   *     shiftedS   = nowS + phaseOffsetS
   *     phaseS     = shiftedS mod periodS
   *
   *   Then the process is ON iff:
   *
   *     phaseS < onS
   *
   * Convention:
   *   - the ON interval is [0, onS)
   *   - exactly at phaseS == onS, the process is considered OFF
   *   - phaseOffsetS shifts the square-wave phase forward before modulo
   *
   * Important:
   *   - this function is pure and does not mutate internal model state
   *   - callers are expected to provide nowS >= 0
   */

  NS_ASSERT(nowS >= 0.0);

  const double onS = std::max(0.0, m_onMeanMs) / 1000.0;
  const double offS = std::max(0.0, m_offMeanMs) / 1000.0;
  const double periodS = onS + offS;

  /*
   * Degenerate fallback:
   * if the total period collapses to zero, treat the process as always ON.
   *
   * In normal operation, ValidateConfiguration() should prevent invalid
   * square-wave configurations when burstiness is enabled.
   */
  if (periodS <= 0.0)
    {
      return true;
    }

  const double phaseOffsetS = std::max(0.0, m_phaseOffsetMs) / 1000.0;
  const double shiftedTimeS = nowS + phaseOffsetS;

  double phaseS = std::fmod(shiftedTimeS, periodS);
  if (phaseS < 0.0)
    {
      phaseS += periodS;
    }

  return (phaseS < onS);
}

void
LpwanAlohaCollisionModel::UpdateToTime(double nowS) const
{
  /*
   * Contract:
   *   Advance the internal bursty ON/OFF process up to time nowS.
   *
   * Behavior:
   *   - if burstiness is disabled:
   *       the process is always ON
   *
   *   - if BurstyModel == "onoff-square":
   *       the ON/OFF state is evaluated directly from the deterministic
   *       square-wave phase at nowS
   *
   *   - otherwise:
   *       the internal stateful ON/OFF process is advanced by consuming
   *       scheduled flip events until the internal process time covers nowS
   *
   * Important:
   *   - For non-square bursty models, this function is stateful and therefore
   *     requires non-decreasing query times to preserve deterministic and
   *     reproducible behavior.
   *
   *   - This function updates:
   *       * m_isOn
   *       * m_tracedIsOn
   *       * m_lastUpdateQueryS
   *
   * Timing semantics:
   *   - m_nextFlipS stores the next scheduled ON/OFF transition time
   *   - the process flips repeatedly until m_nextFlipS > nowS
   *
   * Safety:
   *   - zero-duration or numerically degenerate transitions are guarded
   *     against by enforcing a minimal forward time increment
   */

  NS_ASSERT(nowS >= 0.0);

  EnsureInitialized();

  NS_ABORT_MSG_IF(nowS < m_lastUpdateQueryS,
                  "UpdateToTime requires non-decreasing query times.");

  if (!m_burstyEnabled)
    {
      /*
       * Non-bursty regime: the process is always ON.
       */
      m_isOn = true;
      m_tracedIsOn = true;
      m_lastUpdateQueryS = nowS;
      return;
    }

  RecomputeLambdaOn();

  if (m_burstyModel == "onoff-square")
    {
      /*
       * Deterministic square-wave regime:
       * no state evolution is needed beyond evaluating the phase at nowS.
       */
      m_isOn = ComputeSquareWaveIsOn(nowS);
      m_tracedIsOn = m_isOn;
      m_lastUpdateQueryS = nowS;
      return;
    }

  /*
   * Stateful bursty regime:
   * advance scheduled ON/OFF flips until the internal process time covers
   * nowS.
   */
  while (nowS >= m_nextFlipS)
    {
      if (m_isOn)
        {
          m_isOn = false;
          m_nextFlipS += SampleOffDurationS();
        }
      else
        {
          m_isOn = true;
          m_nextFlipS += SampleOnDurationS();
        }

      /*
       * Defensive safeguard against zero-duration or numerically degenerate
       * transitions that would otherwise stall the loop.
       */
      if (m_nextFlipS <= nowS)
        {
          m_nextFlipS = nowS + 1e-9;
        }
    }

  m_tracedIsOn = m_isOn;
  m_lastUpdateQueryS = nowS;
}

bool
LpwanAlohaCollisionModel::GetIsOnAt(double nowS) const
{
  /*
   * Contract:
   *   Return whether the bursty interference process is in ON state at time
   *   nowS.
   *
   * Behavior:
   *   - if burstiness is disabled, the process is always considered ON
   *   - if BurstyModel == "onoff-square", the ON/OFF state is computed
   *     directly from the deterministic square-wave phase
   *   - otherwise, the stateful bursty process is advanced to time nowS and
   *     the resulting internal ON/OFF state is returned
   *
   * Important:
   *   - For stateful bursty models, this function may update internal state
   *     through UpdateToTime(nowS).
   *   - Therefore reproducibility depends on consistent temporal query order.
   *
   * Side effects:
   *   - updates the exported trace-visible state m_tracedIsOn
   */

  EnsureInitialized();

  bool isOn = true;

  if (!m_burstyEnabled)
    {
      isOn = true;
    }
  else if (m_burstyModel == "onoff-square")
    {
      isOn = ComputeSquareWaveIsOn(nowS);
    }
  else
    {
      UpdateToTime(nowS);
      isOn = m_isOn;
    }

  m_tracedIsOn = isOn;
  return isOn;
}

double
LpwanAlohaCollisionModel::GetEffectiveLambdaTotal(double nowS) const
{
  /*
   * Contract:
   *   Return the effective total interferer attempt rate lambda_total(t)
   *   evaluated at time nowS.
   *
   * Behavior:
   *   - if bursty mode is disabled:
   *
   *       lambda_total(t) = m_lambdaTotalAvg
   *
   *   - if deterministic square-wave bursty mode is enabled:
   *
   *       lambda_total(t) = m_lambdaOnTotal   during ON intervals
   *                       = 0.0               during OFF intervals
   *
   *   - if stateful bursty mode is enabled:
   *       the internal ON/OFF process is advanced to nowS and the effective
   *       lambda is derived from the resulting internal state
   *
   * Important:
   *   - For stateful bursty models, reproducible behavior requires queries at
   *     non-decreasing times.
   *   - This function updates trace-visible state:
   *       * m_tracedIsOn
   *       * m_tracedEffectiveLambdaTotal
   *
   * Notes:
   *   - m_lambdaOnTotal is a derived quantity computed from the configured
   *     long-run average rate and burst duty cycle, so it is recomputed
   *     defensively before evaluation.
   */

  NS_ASSERT(nowS >= 0.0);

  EnsureInitialized();
  RecomputeLambdaOn();

  bool isOn = true;
  double lambdaEff = m_lambdaTotalAvg;

  if (!m_burstyEnabled)
    {
      /*
       * Non-bursty regime: the effective total rate is constant.
       */
      isOn = true;
      lambdaEff = m_lambdaTotalAvg;
    }
  else if (m_burstyModel == "onoff-square")
    {
      /*
       * Deterministic square-wave ON/OFF process.
       */
      isOn = ComputeSquareWaveIsOn(nowS);
      lambdaEff = isOn ? m_lambdaOnTotal : 0.0;
    }
  else
    {
      /*
       * Stateful bursty ON/OFF process.
       *
       * This branch may depend on internal state evolution, so callers are
       * expected to query times in non-decreasing order.
       */
      UpdateToTime(nowS);
      isOn = m_isOn;
      lambdaEff = isOn ? m_lambdaOnTotal : 0.0;
    }

  lambdaEff = std::max(0.0, lambdaEff);

  m_tracedIsOn = isOn;
  m_tracedEffectiveLambdaTotal = lambdaEff;

  NS_LOG_INFO("GET_EFFECTIVE_LAMBDA_TOTAL"
              << " nowS=" << nowS
              << " burstyEnabled=" << m_burstyEnabled
              << " burstyModel=" << m_burstyModel
              << " lambdaTotalAvg=" << m_lambdaTotalAvg
              << " lambdaOnTotal=" << m_lambdaOnTotal
              << " isOn=" << isOn
              << " lambdaEff=" << lambdaEff);

  return lambdaEff;
}

double
LpwanAlohaCollisionModel::GetAverageLambdaTotalOverWindow(double centerS,
                                                          double halfWindowS) const
{
  /*
   * Contract:
   *   Return the average effective total attempt rate lambda_total(t)
   *   over the physically valid portion of the symmetric window
   *
   *       [centerS - halfWindowS, centerS + halfWindowS]
   *
   *   clipped to simulation time t >= 0.
   *
   * Important:
   *   - The simulator time domain is physical and starts at t = 0.
   *   - Therefore, the left edge of the averaging window must be clamped to 0.
   *   - The average is taken over the actual clipped interval width,
   *     not the nominal symmetric width, to avoid bias near t = 0.
   *   - This function must never query GetEffectiveLambdaTotal() with t < 0.
   *   - This function must never return NaN, Inf, or a negative rate.
   */

  NS_ASSERT(centerS >= 0.0);

  const double T = std::max(0.0, halfWindowS);

  /*
   * Degenerate zero-width window -> instantaneous effective rate.
   */
  if (T <= 0.0)
    {
      const double lambda0 = GetEffectiveLambdaTotal(centerS);
      NS_ASSERT(std::isfinite(lambda0));
      NS_ASSERT(lambda0 >= 0.0);
      return lambda0;
    }

  /*
   * Non-bursty case: effective rate is constant over time.
   */
  if (!m_burstyEnabled)
    {
      NS_ASSERT(std::isfinite(m_lambdaTotalAvg));
      NS_ASSERT(m_lambdaTotalAvg >= 0.0);
      return m_lambdaTotalAvg;
    }

  /*
   * Clip the physically valid averaging window to t >= 0.
   */
  const double windowStartS = std::max(0.0, centerS - T);
  const double windowEndS = std::max(windowStartS, centerS + T);
  const double windowWidthS = windowEndS - windowStartS;

  /*
   * Defensive fallback: if clipping degenerates the interval, use the
   * instantaneous rate at the valid center time.
   */
  if (windowWidthS <= 0.0)
    {
      const double lambda0 = GetEffectiveLambdaTotal(centerS);
      NS_ASSERT(std::isfinite(lambda0));
      NS_ASSERT(lambda0 >= 0.0);
      return lambda0;
    }

  /*
   * Midpoint sampling over the clipped interval.
   *
   * A fixed odd number of samples provides stable averaging near ON/OFF
   * transitions while remaining inexpensive.
   */
  const uint32_t nSamples = 17;
  const double dt = windowWidthS / static_cast<double>(nSamples);

  NS_ASSERT(dt > 0.0);
  NS_ASSERT(std::isfinite(dt));

  double sumLambda = 0.0;

  for (uint32_t i = 0; i < nSamples; ++i)
    {
      const double ti =
          windowStartS + (static_cast<double>(i) + 0.5) * dt;

      NS_ASSERT(std::isfinite(ti));
      NS_ASSERT(ti >= 0.0);
      NS_ASSERT(ti >= windowStartS);
      NS_ASSERT(ti <= windowEndS);

      const double lambdaTi = GetEffectiveLambdaTotal(ti);

      NS_ASSERT(std::isfinite(lambdaTi));
      NS_ASSERT(lambdaTi >= 0.0);

      sumLambda += lambdaTi;
    }

  const double avgLambda =
      sumLambda / static_cast<double>(nSamples);

  /*
   * Hard defensive guard:
   * even if future changes corrupt internal helpers, this function must never
   * export an invalid rate to the collision model.
   */
  if (!std::isfinite(avgLambda) || avgLambda < 0.0)
    {
      NS_LOG_ERROR("GET_AVERAGE_LAMBDA_TOTAL_OVER_WINDOW_INVALID"
                   << " centerS=" << centerS
                   << " halfWindowS=" << halfWindowS
                   << " windowStartS=" << windowStartS
                   << " windowEndS=" << windowEndS
                   << " windowWidthS=" << windowWidthS
                   << " sumLambda=" << sumLambda
                   << " avgLambda=" << avgLambda);

      return 0.0;
    }

  NS_LOG_INFO("GET_AVERAGE_LAMBDA_TOTAL_OVER_WINDOW"
              << " centerS=" << centerS
              << " halfWindowS=" << halfWindowS
              << " windowStartS=" << windowStartS
              << " windowEndS=" << windowEndS
              << " windowWidthS=" << windowWidthS
              << " nSamples=" << nSamples
              << " avgLambda=" << avgLambda);

  return avgLambda;
}


double
LpwanAlohaCollisionModel::GetOverlapFractionAt(double nowS,
                                               double airtimeMs,
                                               uint32_t txChannel) const
{
  /*
   * Contract:
   *   Return the fraction of the TX interval
   *
   *       [nowS, nowS + airtimeMs]
   *
   *   that is overlapped by at least one explicit interference event on the
   *   same logical channel txChannel.
   *
   * Behavior:
   *   - if airtimeMs <= 0, return 0
   *   - only events with ev.channel == txChannel are considered
   *   - overlap is computed as the union of overlapping event intervals,
   *     not as a raw sum of per-event overlaps
   *
   * Important:
   *   - simultaneous interferers covering the same sub-interval do NOT
   *     double-count time coverage
   *   - this function assumes m_events is sorted by event start time
   *   - output is guaranteed to be in [0, 1]
   *   - this function must never return NaN or Inf
   */

  NS_ASSERT(nowS >= 0.0);
  NS_ASSERT(airtimeMs >= 0.0);

  const double startS = nowS;
  const double durS = std::max(0.0, airtimeMs) / 1000.0;
  const double endS = startS + durS;

  NS_ASSERT(std::isfinite(startS));
  NS_ASSERT(std::isfinite(durS));
  NS_ASSERT(std::isfinite(endS));
  NS_ASSERT(endS >= startS);

  if (durS <= 0.0)
    {
      return 0.0;
    }

  double coveredS = 0.0;
  bool hasActiveSegment = false;
  double currentStartS = 0.0;
  double currentEndS = 0.0;

  for (const auto& ev : m_events)
    {
      /*
       * Defensive filtering of malformed explicit events.
       *
       * Invalid events are ignored rather than allowed to corrupt overlap
       * accounting.
       */
      if (!std::isfinite(ev.startS) ||
          !std::isfinite(ev.endS) ||
          ev.startS < 0.0 ||
          ev.endS < ev.startS)
        {
          NS_LOG_ERROR("GET_OVERLAP_FRACTION_AT_INVALID_EVENT"
                       << " ev.startS=" << ev.startS
                       << " ev.endS=" << ev.endS
                       << " ev.channel=" << ev.channel
                       << " txChannel=" << txChannel);
          continue;
        }

      /*
       * Since m_events is sorted by start time, events ending before the TX
       * interval can be skipped, and once event start exceeds the TX interval
       * end we may terminate the scan.
       */
      if (ev.endS <= startS)
        {
          continue;
        }

      if (ev.startS >= endS)
        {
          break;
        }

      if (ev.channel != txChannel)
        {
          continue;
        }

      const double ovStart = std::max(startS, ev.startS);
      const double ovEnd = std::min(endS, ev.endS);

      if (!std::isfinite(ovStart) || !std::isfinite(ovEnd))
        {
          NS_LOG_ERROR("GET_OVERLAP_FRACTION_AT_INVALID_OVERLAP"
                       << " ovStart=" << ovStart
                       << " ovEnd=" << ovEnd
                       << " startS=" << startS
                       << " endS=" << endS
                       << " ev.startS=" << ev.startS
                       << " ev.endS=" << ev.endS);
          continue;
        }

      if (ovEnd <= ovStart)
        {
          continue;
        }

      if (!hasActiveSegment)
        {
          currentStartS = ovStart;
          currentEndS = ovEnd;
          hasActiveSegment = true;
          continue;
        }

      if (ovStart <= currentEndS)
        {
          /*
           * Merge overlapping or touching covered sub-intervals.
           */
          currentEndS = std::max(currentEndS, ovEnd);
        }
      else
        {
          /*
           * Close the previous merged interval and start a new one.
           */
          coveredS += (currentEndS - currentStartS);
          currentStartS = ovStart;
          currentEndS = ovEnd;
        }
    }

  if (hasActiveSegment)
    {
      coveredS += (currentEndS - currentStartS);
    }

  /*
   * Defensive numerical hygiene: due to floating-point accumulation, force the
   * covered interval into the physically valid range [0, durS].
   */
  if (!std::isfinite(coveredS))
    {
      NS_LOG_ERROR("GET_OVERLAP_FRACTION_AT_INVALID_COVERED"
                   << " coveredS=" << coveredS
                   << " startS=" << startS
                   << " endS=" << endS
                   << " durS=" << durS
                   << " txChannel=" << txChannel);
      return 0.0;
    }

  coveredS = std::clamp(coveredS, 0.0, durS);

  const double overlapFractionRaw = coveredS / durS;

  if (!std::isfinite(overlapFractionRaw))
    {
      NS_LOG_ERROR("GET_OVERLAP_FRACTION_AT_INVALID_FRACTION"
                   << " coveredS=" << coveredS
                   << " durS=" << durS
                   << " overlapFractionRaw=" << overlapFractionRaw
                   << " txChannel=" << txChannel);
      return 0.0;
    }

  const double overlapFraction =
      std::clamp(overlapFractionRaw, 0.0, 1.0);

  NS_LOG_INFO("GET_OVERLAP_FRACTION_AT"
              << " nowS=" << nowS
              << " airtimeMs=" << airtimeMs
              << " txChannel=" << txChannel
              << " coveredS=" << coveredS
              << " durS=" << durS
              << " overlapFraction=" << overlapFraction);

  return overlapFraction;
}

double
LpwanAlohaCollisionModel::GetCollisionProbabilityAt(double nowS,
                                                    double airtimeMs,
                                                    uint32_t logicalChannels) const
{
  /*
   * Contract:
   *   Return the collision-induced failure probability for one transmission
   *   starting at time nowS with airtime airtimeMs.
   *
   * Behavior:
   *   - if logicalChannels == 0, use the model-configured channel count
   *   - if airtime <= 0 or no channels exist, return 0
   *   - if explicit event-driven collisions are enabled, return a bounded
   *     overlap-based collision proxy
   *   - otherwise use the analytic unslotted-ALOHA approximation:
   *
   *       pCol = 1 - exp(-2 * lambda_ch * T)
   *
   * Important:
   *   - This function must depend on nowS in bursty / time-varying regimes.
   *   - In explicit mode, the returned value is an overlap-based collision
   *     proxy derived from same-channel interference coverage, not a separate
   *     closed-form ALOHA probability.
   *   - In explicit mode, one representative logical channel is evaluated and
   *     channel symmetry is assumed.
   *   - This function must never return NaN, Inf, or a value outside [0,1].
   */

  NS_ASSERT(nowS >= 0.0);
  NS_ASSERT(airtimeMs >= 0.0);

  EnsureInitialized();

  const uint32_t ch =
      (logicalChannels == 0) ? m_logicalChannels : logicalChannels;

  const double airtimeS = std::max(0.0, airtimeMs) / 1000.0;

  NS_ASSERT(std::isfinite(airtimeS));
  NS_ASSERT(airtimeS >= 0.0);

  if (ch == 0 || airtimeS <= 0.0)
    {
      return 0.0;
    }

  if (m_useExplicitEventCollisions)
    {
      if (!m_scheduleBuilt)
        {
          const_cast<LpwanAlohaCollisionModel*>(this)->BuildEventSchedule(nowS + 100.0);
        }

      /*
       * Explicit-mode simplification:
       * evaluate same-channel overlap on one representative logical channel
       * and assume statistical symmetry across channels.
       */
      const uint32_t txChannel = 0;

      const double overlapFractionRaw =
          GetOverlapFractionAt(nowS, airtimeMs, txChannel);

      if (!std::isfinite(overlapFractionRaw))
        {
          NS_LOG_ERROR("GET_COLLISION_PROBABILITY_AT_INVALID_EXPLICIT"
                       << " nowS=" << nowS
                       << " airtimeMs=" << airtimeMs
                       << " logicalChannelsEff=" << ch
                       << " txChannel=" << txChannel
                       << " overlapFractionRaw=" << overlapFractionRaw);
          return 0.0;
        }

      const double overlapFraction =
          std::clamp(overlapFractionRaw, 0.0, 1.0);

      NS_LOG_INFO("GET_COLLISION_PROBABILITY_AT"
                  << " nowS=" << nowS
                  << " airtimeMs=" << airtimeMs
                  << " logicalChannelsEff=" << ch
                  << " mode=explicit"
                  << " txChannel=" << txChannel
                  << " overlapFraction=" << overlapFraction);

      return overlapFraction;
    }

  /*
   * Analytic mode:
   * average the effective total load over the vulnerable window, then project
   * to one logical channel.
   */
  const double lambdaTotalWindow =
      GetAverageLambdaTotalOverWindow(nowS, airtimeS);

  if (!std::isfinite(lambdaTotalWindow))
    {
      NS_LOG_ERROR("GET_COLLISION_PROBABILITY_AT_INVALID_ANALYTIC_LAMBDA"
                   << " nowS=" << nowS
                   << " airtimeMs=" << airtimeMs
                   << " logicalChannelsEff=" << ch
                   << " lambdaTotalWindow=" << lambdaTotalWindow);
      return 0.0;
    }

  if (lambdaTotalWindow <= 0.0)
    {
      return 0.0;
    }

  const double lambdaCh =
      lambdaTotalWindow / static_cast<double>(ch);

  if (!std::isfinite(lambdaCh) || lambdaCh < 0.0)
    {
      NS_LOG_ERROR("GET_COLLISION_PROBABILITY_AT_INVALID_ANALYTIC_CHANNEL_RATE"
                   << " nowS=" << nowS
                   << " airtimeMs=" << airtimeMs
                   << " logicalChannelsEff=" << ch
                   << " lambdaTotalWindow=" << lambdaTotalWindow
                   << " lambdaCh=" << lambdaCh);
      return 0.0;
    }

  const double exponent = -2.0 * lambdaCh * airtimeS;

  if (!std::isfinite(exponent))
    {
      NS_LOG_ERROR("GET_COLLISION_PROBABILITY_AT_INVALID_EXPONENT"
                   << " nowS=" << nowS
                   << " airtimeMs=" << airtimeMs
                   << " logicalChannelsEff=" << ch
                   << " lambdaCh=" << lambdaCh
                   << " airtimeS=" << airtimeS
                   << " exponent=" << exponent);
      return 0.0;
    }

  const double pColRaw = 1.0 - std::exp(exponent);

  if (!std::isfinite(pColRaw))
    {
      NS_LOG_ERROR("GET_COLLISION_PROBABILITY_AT_INVALID_ANALYTIC_PCOL"
                   << " nowS=" << nowS
                   << " airtimeMs=" << airtimeMs
                   << " logicalChannelsEff=" << ch
                   << " lambdaTotalWindow=" << lambdaTotalWindow
                   << " lambdaCh=" << lambdaCh
                   << " airtimeS=" << airtimeS
                   << " exponent=" << exponent
                   << " pColRaw=" << pColRaw);
      return 0.0;
    }

  const double pCol =
      std::clamp(pColRaw, 0.0, 1.0);

  NS_LOG_INFO("GET_COLLISION_PROBABILITY_AT"
              << " nowS=" << nowS
              << " airtimeMs=" << airtimeMs
              << " logicalChannelsEff=" << ch
              << " mode=analytic"
              << " lambdaTotalWindow=" << lambdaTotalWindow
              << " lambdaCh=" << lambdaCh
              << " exponent=" << exponent
              << " pCol=" << pCol);

  return pCol;
}

void
LpwanAlohaCollisionModel::BuildEventSchedule(double horizonS) const
{
  /*
   * Contract:
   *   Build an explicit event-driven interference schedule over [0, horizonS].
   *
   * Behavior:
   *   - events are generated according to the effective instantaneous load
   *     returned by GetEffectiveLambdaTotal(t)
   *   - each event is assigned one logical channel uniformly at random
   *   - event airtime is fixed to InterfererAirtimeMs
   *
   * Important:
   *   This schedule is built from the current internal bursty-state evolution.
   *   For stateful bursty models, deterministic reproducibility therefore
   *   depends on consistent initialization and query ordering.
   */

  m_events.clear();
  m_scheduleBuilt = false;

  EnsureInitialized();

  const double effectiveHorizonS = std::max(0.0, horizonS);
  const double interfererAirtimeS =
      std::max(0.0, m_interfererAirtimeMs) / 1000.0;

  NS_ASSERT(m_channelRv != nullptr);
  NS_ASSERT(m_arrivalRv != nullptr);
  NS_ASSERT(m_logicalChannels > 0);
  NS_ABORT_MSG_IF(interfererAirtimeS <= 0.0,
                  "BuildEventSchedule requires InterfererAirtimeMs > 0.");

  if (effectiveHorizonS <= 0.0)
    {
      m_scheduleBuilt = true;
      return;
    }

  NS_LOG_INFO("BUILD_EVENT_SCHEDULE_BEGIN"
              << " horizonS=" << effectiveHorizonS
              << " interfererAirtimeS=" << interfererAirtimeS
              << " logicalChannels=" << m_logicalChannels
              << " burstyEnabled=" << m_burstyEnabled
              << " burstyModel=" << m_burstyModel
              << " lambdaTotalAvg=" << m_lambdaTotalAvg);

  double t = 0.0;
  const double idleAdvanceS = 1e-3;

  while (t < effectiveHorizonS)
    {
      const double lambda = GetEffectiveLambdaTotal(t);

      if (lambda <= 0.0)
        {
          /*
           * No arrivals can occur while the effective total attempt rate is zero.
           * Advance time conservatively to avoid a tight loop.
           */
          t += idleAdvanceS;
          continue;
        }

      m_arrivalRv->SetAttribute("Mean", DoubleValue(1.0 / lambda));
      const double dt = std::max(m_arrivalRv->GetValue(), 1e-9);

      t += dt;

      if (t >= effectiveHorizonS)
        {
          break;
        }

      InterferenceEvent ev;
      ev.startS = t;
      ev.endS = t + interfererAirtimeS;
      ev.channel = m_channelRv->GetInteger(0, m_logicalChannels - 1);

      m_events.push_back(ev);
    }

  std::sort(m_events.begin(), m_events.end(),
            [](const InterferenceEvent& a, const InterferenceEvent& b)
            {
              return a.startS < b.startS;
            });

  m_scheduleBuilt = true;

  NS_LOG_INFO("BUILD_EVENT_SCHEDULE_END"
              << " nEvents=" << m_events.size()
              << " scheduleBuilt=" << m_scheduleBuilt);
}

void
LpwanAlohaCollisionModel::SetUseExplicitEventCollisions(bool enable)
{
  /*
   * Contract:
   *   Enable or disable explicit event-driven collision modeling.
   *
   * Behavior:
   *   - if enable == true:
   *       collision probability will be computed from an explicit
   *       interference event schedule (event-driven model)
   *
   *   - if enable == false:
   *       collision probability will be computed using the analytic
   *       unslotted ALOHA approximation
   *
   *           pCol = 1 - exp(-2 * lambda_ch * T)
   *
   * Important:
   *   - Switching this flag changes the semantic interpretation of the
   *     collision model:
   *
   *       * analytic mode → stationary approximation
   *       * explicit mode → time-resolved, phase-dependent interference
   *
   *   - This flag should be configured before simulation starts.
   *     Changing it at runtime may invalidate previously built schedules.
   */
   
  if (m_useExplicitEventCollisions != enable)
    {
      NS_LOG_INFO("SET_EXPLICIT_COLLISION_MODE"
                  << " old=" << m_useExplicitEventCollisions
                  << " new=" << enable);

      /*
       * Invalidate previously built schedule to avoid stale interference traces.
       */
      m_scheduleBuilt = false;
      m_events.clear();
    }

  m_useExplicitEventCollisions = enable;
}

bool
LpwanAlohaCollisionModel::GetUseExplicitEventCollisions() const
{
  /*
   * Contract:
   *   Return whether the collision model is currently operating in
   *   explicit event-driven mode.
   *
   * Returns:
   *   true  → explicit event-driven collision model is enabled
   *   false → analytic ALOHA-based collision model is used
   *
   * Usage:
   *   This accessor is typically used for:
   *     - debugging and logging
   *     - exporting configuration to result datasets
   *     - ensuring reproducibility of simulation conditions
   */

  return m_useExplicitEventCollisions;
}

void
LpwanAlohaCollisionModel::ValidateConfiguration() const
{
  /*
   * Contract:
   *   Validate the public configuration of the collision model before use.
   *
   * Philosophy:
   *   - reject invalid bursty regimes
   *   - reject impossible average-load preservation states
   *   - keep explicit-event and analytic modes internally consistent
   */

  // --------------------------------------------------------------------------
  // Core load / channels
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(m_lambdaTotalAvg < 0.0,
                  "Invalid LambdaTotalAvg: must be >= 0 attempts/s.");

  NS_ABORT_MSG_IF(m_logicalChannels == 0,
                  "Invalid LogicalChannels: must be >= 1.");

  // --------------------------------------------------------------------------
  // Bursty model selection
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(!(m_burstyModel == "onoff-exp" ||
                    m_burstyModel == "onoff-square"),
                  "Invalid BurstyModel: supported values are 'onoff-exp' and 'onoff-square'.");

  NS_ABORT_MSG_IF(m_onMeanMs < 0.0 || m_offMeanMs < 0.0,
                  "Invalid bursty durations: OnMeanMs and OffMeanMs must be >= 0.");

  NS_ABORT_MSG_IF(m_phaseOffsetMs < 0.0,
                  "Invalid PhaseOffsetMs: must be >= 0.");

  if (m_burstyEnabled)
    {
      if (m_burstyModel == "onoff-square")
        {
          NS_ABORT_MSG_IF((m_onMeanMs + m_offMeanMs) <= 0.0,
                          "Invalid onoff-square configuration: OnMeanMs + OffMeanMs must be > 0 to define a real period.");

          NS_ABORT_MSG_IF(m_onMeanMs <= 0.0 && m_lambdaTotalAvg > 0.0,
                          "Invalid onoff-square configuration: positive LambdaTotalAvg requires OnMeanMs > 0 so the process can actually be ON.");
        }
      else if (m_burstyModel == "onoff-exp")
        {
          NS_ABORT_MSG_IF(m_onMeanMs <= 0.0,
                          "Invalid onoff-exp configuration: OnMeanMs must be > 0.");

          NS_ABORT_MSG_IF(m_offMeanMs <= 0.0,
                          "Invalid onoff-exp configuration: OffMeanMs must be > 0.");
        }
    }

  // --------------------------------------------------------------------------
  // Average-load preservation consistency
  // --------------------------------------------------------------------------
  if (m_burstyEnabled)
    {
      const double onS = std::max(0.0, m_onMeanMs) / 1000.0;
      const double offS = std::max(0.0, m_offMeanMs) / 1000.0;
      const double periodS = onS + offS;
      const double duty = (periodS > 0.0) ? (onS / periodS) : 1.0;

      NS_ABORT_MSG_IF(duty <= 0.0 && m_lambdaTotalAvg > 0.0,
                      "Invalid bursty configuration: ON duty cycle is zero while LambdaTotalAvg is positive. The model cannot preserve the configured average load.");
    }

  // --------------------------------------------------------------------------
  // Explicit-event collision mode
  // --------------------------------------------------------------------------
  if (m_useExplicitEventCollisions)
    {
      NS_ABORT_MSG_IF(m_interfererAirtimeMs <= 0.0,
                      "Invalid explicit-event collision configuration: InterfererAirtimeMs must be > 0 when UseExplicitEventCollisions=true.");
    }
  else
    {
      NS_ABORT_MSG_IF(m_interfererAirtimeMs < 0.0,
                      "Invalid InterfererAirtimeMs: must be >= 0.");
    }
}

} // namespace ns3
