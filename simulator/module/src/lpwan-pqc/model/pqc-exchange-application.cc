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
 * ============================================================================
 * SIMULATOR CONTRACT — PQC LoRaWAN Exchange Simulator
 * ============================================================================
 *
 * This simulator models the execution of a full PQC key exchange over a
 * LoRa-like LPWAN link using a fragment-based transmission protocol.
 *
 * The implementation follows a strict separation of concerns between
 * PHY/channel behavior, collision processes, receiver availability, and
 * application-level retransmission logic.
 *
 * ---------------------------------------------------------------------------
 * 1. Exchange Model
 * ---------------------------------------------------------------------------
 *
 * A single simulation step corresponds to one "exchange", defined as the
 * complete transmission of:
 *
 *   - pkFragments   (public key fragments)
 *   - ctFragments   (ciphertext fragments)
 *
 * Total fragments:
 *
 *   nFragmentsExchange = pkFragments + ctFragments
 *
 * Each fragment may require multiple attempts due to channel failures,
 * collisions, or receiver unavailability.
 *
 * ---------------------------------------------------------------------------
 * 2. Failure Model
 * ---------------------------------------------------------------------------
 *
 * Fragment transmission failures arise from three independent causes:
 *
 *   1. PHY/channel decoding failure
 *   2. Collision with concurrent transmissions
 *   3. Receiver sleep (RX duty-cycle unavailable)
 *
 * The simulator records these outcomes separately to allow causal analysis.
 *
 * The total failure probability used for Monte-Carlo simulation is:
 *
 *   p_total = 1 - (1 - p_phy) * (1 - p_collision)
 *
 * Receiver sleep is evaluated deterministically via RX window scheduling.
 *
 * ---------------------------------------------------------------------------
 * 3. Collision Model
 * ---------------------------------------------------------------------------
 *
 * Collisions are modeled through LpwanAlohaCollisionModel.
 *
 * Two modes are supported:
 *
 *   - Analytical collision probability
 *   - Explicit event-driven collisions
 *
 * The canonical API is:
 *
 *   GetCollisionProbabilityAt(time, airtime, logicalChannels)
 *
 * The collision model is the sole authority responsible for collision
 * semantics.
 *
 * ---------------------------------------------------------------------------
 * 4. Receiver Duty Cycle Model
 * ---------------------------------------------------------------------------
 *
 * When RX duty cycling is enabled, the receiver alternates between:
 *
 *   RX window  : duration rxWindowMs
 *   RX idle    : duration rxIdleMs
 *
 * A frame is considered successfully received **only if its entire airtime
 * interval lies fully inside an active RX window**.
 *
 * Partial overlaps with RX windows do NOT count as successful reception.
 *
 * This avoids optimistic artifacts where long frames would otherwise be
 * accepted due to minimal overlap.
 *
 * ---------------------------------------------------------------------------
 * 5. Fragment Attempts and Retransmissions
 * ---------------------------------------------------------------------------
 *
 * Each fragment may be retransmitted up to:
 *
 *   maxRetries
 *
 * Retransmission timing is controlled by the pacing policy:
 *
 *   - baseline
 *   - radio-aware pacing
 *   - phase-aware pacing
 *
 * Attempt timestamps therefore influence collision probability and burst
 * phase alignment.
 *
 * ---------------------------------------------------------------------------
 * 6. ACK Batching
 * ---------------------------------------------------------------------------
 *
 * Fragment acknowledgments may be batched:
 *
 *   ackBatchSize ≥ 1
 *
 * ACK failures may trigger:
 *
 *   - logical ACK failures
 *   - ACK retransmission rounds
 *   - redelivery of previously accepted fragments
 *
 * ---------------------------------------------------------------------------
 * 7. Exchange Result Contract
 * ---------------------------------------------------------------------------
 *
 * Each exchange produces exactly one ExchangeResult record.
 *
 * The CSV export is a 1:1 serialization of this structure.
 *
 * Core invariants:
 *
 *   okFrags + failFrags = nFragmentsExchange
 *
 *   framesTxTotal = dataFramesTx + ackFramesTx
 *
 *   attemptsTotal = attemptsOn + attemptsOff
 *
 * All probability metrics are constrained to:
 *
 *   0 ≤ p ≤ 1
 *
 * ---------------------------------------------------------------------------
 * 8. Logging Policy
 * ---------------------------------------------------------------------------
 *
 * Logging is restricted to a minimal set of contractual events:
 *
 *   INITIALIZE_EXCHANGE_RESULT
 *   ATTEMPT_BEGIN
 *   ATTEMPT_RESULT
 *   APPLY_FRAGMENT_GAP
 *   APPLY_DATA_ATTEMPT_GAP
 *   APPLY_ACK_RECOVERY_GAP
 *   FINALIZE_EXCHANGE_OUT
 *
 * This ensures deterministic and interpretable traces for debugging
 * and experimental validation.
 *
 * ---------------------------------------------------------------------------
 * 9. Determinism
 * ---------------------------------------------------------------------------
 *
 * Simulation runs are deterministic given:
 *
 *   (seed, run)
 *
 * All stochastic behavior derives from ns-3 RNG streams.
 *
 * ---------------------------------------------------------------------------
 * 10. Output Stability
 * ---------------------------------------------------------------------------
 *
 * The CSV schema is considered part of the simulator contract.
 *
 * Column order and semantics must remain stable to ensure reproducibility
 * of experimental datasets and figures.
 *
 * ============================================================================
 */
 
// Reception model:
// A frame is considered received only if its entire airtime interval
// is contained within an active RX window. Partial overlaps do not
// count as successful reception, preventing optimistic duty-cycle
// artifacts where long frames would otherwise be accepted.

#include "ns3/pqc-exchange-application.h"

#include "ns3/lpwan-aloha-collision-model.h"
#include "ns3/lpwan-lora-airtime.h"
#include "ns3/lpwan-packet-error-model.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("PqcExchangeApplication");
NS_OBJECT_ENSURE_REGISTERED(PqcExchangeApplication);

//------------------------------------------------------------------------------
// TypeId / Attributes
//------------------------------------------------------------------------------

TypeId
PqcExchangeApplication::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::PqcExchangeApplication")
          .SetParent<Application>()
          .SetGroupName("LpwanPqc")
          .AddConstructor<PqcExchangeApplication>()

          // ------------------------------------------------------------------
          // Experiment identity / scenario labeling
          // ------------------------------------------------------------------
          .AddAttribute("Environment",
                        "Scenario label used to select the baseline SNR regime "
                        "(e.g., indoor, outdoor).",
                        StringValue("indoor"),
                        MakeStringAccessor(&PqcExchangeApplication::m_environment),
                        MakeStringChecker())

          .AddAttribute("Sf",
                        "LoRa spreading factor used by the exchange.",
                        UintegerValue(7),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_sf),
                        MakeUintegerChecker<uint8_t>(7, 12))

          .AddAttribute("Variant",
                        "Experiment variant label exported to the CSV output.",
                        StringValue("baseline"),
                        MakeStringAccessor(&PqcExchangeApplication::m_variant),
                        MakeStringChecker())

          // ------------------------------------------------------------------
          // Fragmentation / payload contract
          // ------------------------------------------------------------------
          .AddAttribute("PkFragments",
                        "Configured number of PK fragments when AutoFragments=false.",
                        UintegerValue(6),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_pkFragments),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("CtFragments",
                        "Configured number of CT fragments when AutoFragments=false.",
                        UintegerValue(9),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_ctFragments),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("PayloadBytes",
                        "Configured crypto payload bytes per fragment.",
                        UintegerValue(51),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_payloadBytes),
                        MakeUintegerChecker<uint32_t>(1))

          .AddAttribute("AutoFragments",
                        "If true, derive PK/CT fragment counts automatically from "
                        "PkBytes, CtBytes, PayloadBytes, G2gOverheadBytes and the "
                        "PHY payload cap.",
                        BooleanValue(false),
                        MakeBooleanAccessor(&PqcExchangeApplication::m_autoFragments),
                        MakeBooleanChecker())

          .AddAttribute("PkBytes",
                        "Total PK bytes used when AutoFragments=true.",
                        UintegerValue(800),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_pkBytes),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("CtBytes",
                        "Total CT bytes used when AutoFragments=true.",
                        UintegerValue(768),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_ctBytes),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("MaxPayloadOverrideBytes",
                        "Optional PHY payload cap override used only when "
                        "AutoFragments=true. A value of 0 means: use the "
                        "SF-dependent LoRa maximum payload.",
                        UintegerValue(0),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_maxPayloadOverrideBytes),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("G2gOverheadBytes",
                        "Gateway-to-gateway protocol overhead in bytes added on top "
                        "of the crypto fragment payload.",
                        UintegerValue(28),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_g2gOverheadBytes),
                        MakeUintegerChecker<uint32_t>())

          // ------------------------------------------------------------------
          // Pacing policy (core public knobs only)
          // ------------------------------------------------------------------
          .AddAttribute("Pacing",
                        "Pacing mode. Supported values: "
                        "baseline, fixed, radioaware, radioaware_phase_bounded, stochastic.",
                        StringValue("baseline"),
                        MakeStringAccessor(&PqcExchangeApplication::SetPacing,
                                           &PqcExchangeApplication::GetPacing),
                        MakeStringChecker())

          .AddAttribute("FixedGapMs",
                        "Fixed inter-fragment / retry gap in milliseconds used when "
                        "Pacing=fixed.",
                        UintegerValue(0),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_fixedGapMs),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("KTsym",
                        "Multiplier applied to LoRa symbol time in radio-aware pacing.",
                        DoubleValue(4.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_kTsym),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("SfAwarePacingThreshold",
                        "Minimum SF at which radio-aware pacing becomes active.",
                        UintegerValue(255),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_sfAwarePacingThreshold),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("GapMinMs",
                       "Minimum bounded pacing gap in milliseconds used by "
                       "radioaware_phase_bounded pacing.",
                       UintegerValue(0),
                       MakeUintegerAccessor(&PqcExchangeApplication::m_gapMinMs),
                       MakeUintegerChecker<uint32_t>())

          .AddAttribute("GapMaxMs",
                       "Maximum bounded pacing gap in milliseconds used by "
                       "radioaware_phase_bounded pacing. Under RX duty cycling, "
                       "this must be >= RxWindowMs + RxIdleMs so retries can "
                       "escape bad RX phases.",
                       UintegerValue(200),
                       MakeUintegerAccessor(&PqcExchangeApplication::m_gapMaxMs),
                       MakeUintegerChecker<uint32_t>())

          .AddAttribute("RetryMarginDb",
                        "Retry margin exported in the CSV and used by retry pacing "
                        "severity logic.",
                        DoubleValue(1.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_retryMarginDb),
                        MakeDoubleChecker<double>(0.0))
                        
          

          // ------------------------------------------------------------------
          // Retry / ACK / RX duty-cycle behavior
          // ------------------------------------------------------------------
          .AddAttribute("MaxRetries",
                        "Maximum number of retransmissions allowed per DATA fragment.",
                        UintegerValue(2),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_maxRetries),
                        MakeUintegerChecker<uint32_t>())

          .AddAttribute("RxDutyCycleEnabled",
                        "If true, DATA and ACK reception require full containment "
                        "inside active RX windows.",
                        BooleanValue(false),
                        MakeBooleanAccessor(&PqcExchangeApplication::m_rxDutyCycleEnabled),
                        MakeBooleanChecker())

          .AddAttribute("RxWindowMs",
                        "Active RX window duration in milliseconds.",
                        DoubleValue(0.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_rxWindowMs),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("RxIdleMs",
                        "RX idle duration in milliseconds between active windows.",
                        DoubleValue(0.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_rxIdleMs),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("AckBatchSize",
                        "Number of DATA-delivered fragments grouped under one logical "
                        "ACK batch.",
                        UintegerValue(1),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_ackBatchSize),
                        MakeUintegerChecker<uint32_t>(1))

          // ------------------------------------------------------------------
          // Scenario SNR / link-budget model
          // ------------------------------------------------------------------
          .AddAttribute("SnrIndoorDb",
                        "Scenario SNR used when Environment=indoor and "
                        "UseLinkBudget=false.",
                        DoubleValue(-3.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_snrIndoorDb),
                        MakeDoubleChecker<double>())

          .AddAttribute("SnrOutdoorDb",
                        "Scenario SNR used when Environment=outdoor and "
                        "UseLinkBudget=false.",
                        DoubleValue(8.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_snrOutdoorDb),
                        MakeDoubleChecker<double>())

          .AddAttribute("UseLinkBudget",
                        "If true, compute SNR from link-budget parameters instead of "
                        "using the scenario SNR constants.",
                        BooleanValue(false),
                        MakeBooleanAccessor(&PqcExchangeApplication::m_useLinkBudget),
                        MakeBooleanChecker())

          .AddAttribute("TxPowerDbm",
                        "Transmit power in dBm used by the link-budget model.",
                        DoubleValue(14.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_txPowerDbm),
                        MakeDoubleChecker<double>())

          .AddAttribute("DistanceM",
                        "Link distance in meters used by the link-budget model.",
                        DoubleValue(1000.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_distanceM),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("RefDistanceM",
                        "Reference distance in meters used by the path-loss model.",
                        DoubleValue(1.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_refDistanceM),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("RefPathLossDb",
                        "Path loss at the reference distance, in dB.",
                        DoubleValue(32.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_refPathLossDb),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("PathLossExp",
                        "Path-loss exponent used by the link-budget model.",
                        DoubleValue(3.5),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_pathLossExp),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("ShadowingSigmaDb",
                        "Log-normal shadowing sigma in dB.",
                        DoubleValue(0.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_shadowingSigmaDb),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("NoiseFigureDb",
                        "Receiver noise figure in dB.",
                        DoubleValue(6.0),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_noiseFigureDb),
                        MakeDoubleChecker<double>(0.0))

          // ------------------------------------------------------------------
          // LoRa PHY / airtime configuration
          // ------------------------------------------------------------------
          .AddAttribute("BwHz",
                        "Bandwidth in Hz used by airtime and sensitivity calculations.",
                        UintegerValue(125000),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_bwHz),
                        MakeUintegerChecker<uint32_t>(1))

          .AddAttribute("CrDen",
                        "Denominator of coding rate CR=4/CrDen.",
                        UintegerValue(8),
                        MakeUintegerAccessor(&PqcExchangeApplication::m_crDen),
                        MakeUintegerChecker<uint8_t>(5, 8))

          // ------------------------------------------------------------------
          // Energy model
          // ------------------------------------------------------------------
          .AddAttribute("TxEnergyJPerMs",
                        "Transmit energy in joules per millisecond of TX airtime.",
                        DoubleValue(0.0004),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_txEnergyJPerMs),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("RxEnergyJPerMs",
                        "Receive/listening energy in joules per millisecond of RX time.",
                        DoubleValue(0.0002),
                        MakeDoubleAccessor(&PqcExchangeApplication::m_rxEnergyJPerMs),
                        MakeDoubleChecker<double>(0.0))

          .AddAttribute("RxEnergyUsesExchangeLatency",
                        "If true and RX duty cycle is disabled, RX energy is computed "
                        "from exchange latency instead of TX airtime.",
                        BooleanValue(true),
                        MakeBooleanAccessor(&PqcExchangeApplication::m_rxEnergyUsesExchangeLatency),
                        MakeBooleanChecker());

  return tid;
}

PqcExchangeApplication::PqcExchangeApplication()
{
  m_uv = CreateObject<UniformRandomVariable>();

  m_shadowRv = CreateObject<NormalRandomVariable>();
  m_shadowRv->SetAttribute("Mean", DoubleValue(0.0));
  m_shadowRv->SetAttribute("Variance", DoubleValue(1.0));
}

PqcExchangeApplication::~PqcExchangeApplication() = default;

void
PqcExchangeApplication::SetCollisionModel(Ptr<LpwanAlohaCollisionModel> model)
{
  m_col = model;
}

void
PqcExchangeApplication::SetPacketErrorModel(Ptr<LpwanPacketErrorModel> model)
{
  m_per = model;

}

double
PqcExchangeApplication::GetScenarioSnrDb() const
{
  const double snrDb =
      m_useLinkBudget
          ? ComputeSnrDb(m_distanceM)
          : ((m_environment == "indoor") ? m_snrIndoorDb : m_snrOutdoorDb);

  return snrDb;
}

uint32_t
PqcExchangeApplication::ComputePhyPayloadBytes(uint32_t payloadCrypto) const
{
  const uint32_t payloadPhy = payloadCrypto + m_g2gOverheadBytes;

  return payloadPhy;
}

void
PqcExchangeApplication::InitializeExchangeMetadata(ExchangeContext& ctx) const
{
  /*
   * Contract:
   *   Populate the exported metadata snapshot for this exchange before protocol
   *   execution starts.
   *
   * Responsibilities:
   *   - copy experiment identity fields
   *   - snapshot initial burst/interference diagnostics
   *   - resolve exported fragmentation metadata
   *   - resolve configured vs effective payload sizes
   *   - snapshot static exchange configuration parameters
   *
   * Important:
   *   This method does NOT build the actual per-fragment payload vector.
   *   The real fragment plan is built separately by BuildFragmentPlan(...).
   */

  /*
   * Experiment identity fields.
   */
  ctx.out.environment = m_environment;
  ctx.out.sf = m_sf;
  ctx.out.variant = m_variant;
  /*
  * Export pacing mode as part of the stable experiment-configuration snapshot.
  */
  
  ctx.out.pacingMode = GetPacing();
  
  /*
   * Static exchange-configuration snapshot.
   *
   * These values are configuration metadata, not runtime results, so they are
   * initialized once here and must not be overwritten during finalization.
   */
  ctx.out.sfAwarePacingThreshold = m_sfAwarePacingThreshold;
  ctx.out.retryMarginDb = m_retryMarginDb;
  ctx.out.rxDutyCycleEnabled = m_rxDutyCycleEnabled;
  ctx.out.rxWindowMs = m_rxWindowMs;
  ctx.out.rxIdleMs = m_rxIdleMs;
  ctx.out.ackBatchSize = m_ackBatchSize;

  /*
   * Snapshot initial interference/bursty state at exchange start time.
   *
   * If no collision model is installed, lambdaEff is exported as 0 and isOn as
   * 0 (not active / not applicable in practice).
   */
  if (m_col)
    {
      ctx.out.lambdaEff = m_col->GetEffectiveLambdaTotal(ctx.runtime.exchangeStartS);
      ctx.out.isOn = m_col->GetIsOnAt(ctx.runtime.exchangeStartS) ? 1u : 0u;
    }
  else
    {
      ctx.out.lambdaEff = 0.0;
      ctx.out.isOn = 0u;
    }

  /*
   * Exported fragmentation metadata.
   *
   * Two modes are supported:
   *
   *   1. AutoFragments = true
   *      - payloadBytesConfig acts as a configured upper bound for crypto
   *        payload per fragment
   *      - payloadBytesEffective is the actual crypto payload cap after PHY cap
   *        and G2G overhead are applied
   *      - exported pkFragments/ctFragments are derived from total PK/CT bytes
   *
   *   2. AutoFragments = false
   *      - exported pkFragments/ctFragments come directly from configured
   *        fragment counts
   *      - payloadBytesEffective equals payloadBytesConfig
   */
  if (m_autoFragments)
    {
      /*
       * Conservative PHY payload cap used by this simulator.
       */
      uint32_t capPhy =
          (m_maxPayloadOverrideBytes > 0)
              ? m_maxPayloadOverrideBytes
              : std::max<uint32_t>(1, LpwanLoraAirtime::MaxPayloadBytes(m_sf));

      capPhy = std::max<uint32_t>(1, capPhy);

      /*
       * payloadBytesConfig is still required as a configured crypto-payload cap
       * even in auto-fragmentation mode.
       */
      NS_ABORT_MSG_IF(m_payloadBytes == 0,
                      "PayloadBytes must be > 0 when AutoFragments=true");

      /*
       * Effective crypto payload per fragment cannot exceed the PHY cap minus
       * G2G overhead.
       */
      const uint32_t maxCryptoPayload =
          (capPhy > m_g2gOverheadBytes)
              ? (capPhy - m_g2gOverheadBytes)
              : 1u;

      const uint32_t effectivePayloadBytes =
          std::min<uint32_t>(m_payloadBytes, maxCryptoPayload);

      ctx.out.payloadBytesConfig = m_payloadBytes;
      ctx.out.payloadBytesEffective = effectivePayloadBytes;

      /*
       * Export derived logical fragment counts.
       */
      ctx.out.pkFragments =
          (m_pkBytes > 0)
              ? static_cast<uint32_t>((m_pkBytes + effectivePayloadBytes - 1) /
                                      effectivePayloadBytes)
              : 0u;

      ctx.out.ctFragments =
          (m_ctBytes > 0)
              ? static_cast<uint32_t>((m_ctBytes + effectivePayloadBytes - 1) /
                                      effectivePayloadBytes)
              : 0u;
    }
  else
    {
      /*
       * Manual fragmentation mode:
       * use the configured fragment counts and configured payload directly.
       */
      ctx.out.pkFragments = m_pkFragments;
      ctx.out.ctFragments = m_ctFragments;
      ctx.out.payloadBytesConfig = m_payloadBytes;
      ctx.out.payloadBytesEffective = m_payloadBytes;
    }

  /*
   * Export G2G overhead snapshot used by this exchange.
   */
  ctx.out.g2gOverheadBytes = m_g2gOverheadBytes;
}

double
PqcExchangeApplication::GetSnrMinDb(uint32_t sf)
{
  switch (sf)
    {
    case 7:  return -7.5;
    case 8:  return -10.0;
    case 9:  return -12.5;
    case 10: return -15.0;
    case 11: return -17.5;
    case 12: return -20.0;
    default:
      NS_LOG_WARN("GET_SNR_MIN_DB"
                  << " unsupportedSf=" << sf
                  << " fallbackSf=7");
      return -7.5;
    }
}

void
PqcExchangeApplication::InitializeScenarioDiagnostics(ExchangeContext& ctx) const
{
  /*
   * Contract:
   *   Populate scenario/link diagnostics exported for this exchange before
   *   protocol execution starts.
   *
   * Responsibilities:
   *   - snapshot the operating SNR used by the exchange
   *   - snapshot thermal-noise floor
   *   - derive received power and path loss when link-budget mode is enabled
   *   - derive link margin against LoRa sensitivity
   *
   * Important:
   *   These fields are diagnostics exported with the exchange result.
   *   They do not, by themselves, execute PHY failure or collision logic.
   */

  /*
   * Operating SNR snapshot for this exchange.
   *
   * Depending on simulator configuration, this may come from:
   *   - fixed environment baseline SNR, or
   *   - link-budget-derived SNR logic elsewhere in the model.
   */
  ctx.out.snrDb = GetScenarioSnrDb();

  /*
   * Receiver noise floor snapshot in dBm.
   */
  ctx.out.noiseDbm = ComputeNoiseDbm();

  if (m_useLinkBudget)
    {
      /*
       * Explicit link-budget mode:
       * derive path loss and received power from the configured propagation
       * model and TX power.
       */
      ctx.out.pathLossDb = ComputePathLossDb(m_distanceM);
      ctx.out.rxDbm = m_txPowerDbm - ctx.out.pathLossDb;
    }
  else
    {
      /*
       * Baseline-SNR mode:
       * path loss is not modeled explicitly in this mode, so export 0.0 as a
       * sentinel value and reconstruct received power consistently from:
       *
       *   rxDbm = snrDb + noiseDbm
       */
      ctx.out.pathLossDb = 0.0;
      ctx.out.rxDbm = ctx.out.snrDb + ctx.out.noiseDbm;
    }

  /*
   * Sensitivity-based link margin snapshot.
   *
   * This is the difference between received power and estimated LoRa receiver
   * sensitivity for the configured SF/bandwidth/noise figure.
   */
  const double sensDbm = GetSensitivityDbm(m_sf, m_bwHz, m_noiseFigureDb);
  ctx.out.marginDb = ctx.out.rxDbm - sensDbm;
}

void
PqcExchangeApplication::AccumulateNominalFragmentStats(ExchangeContext& ctx,
                                                       uint32_t payloadPhy,
                                                       double airtimeFragMs,
                                                       double airtimeFragS,
                                                       double pPhy)
{
  /*
   * Contract:
   *   Accumulate fragment-level nominal statistics that depend only on the
   *   fragment payload and baseline scenario parameters, not on the realized
   *   attempt schedule.
   *
   * Responsibilities:
   *   - accumulate nominal PHY/channel failure probability
   *   - accumulate fragment PHY payload size
   *   - accumulate fragment airtime
   *
   * Important:
   *   Scheduling-dependent collision exposure MUST NOT be accumulated here.
   */

  NS_ASSERT(std::isfinite(airtimeFragMs));
  NS_ASSERT(std::isfinite(airtimeFragS));
  NS_ASSERT(std::isfinite(pPhy));

  NS_ASSERT(airtimeFragMs >= 0.0);
  NS_ASSERT(airtimeFragS >= 0.0);
  NS_ASSERT(pPhy >= 0.0 && pPhy <= 1.0);

  ctx.stats.sumPhy += pPhy;
  ctx.stats.sumPhyPayloadBytes += static_cast<double>(payloadPhy);
  ctx.stats.airtimeFragMsSum += airtimeFragMs;

  NS_ASSERT(std::isfinite(ctx.stats.sumPhy));
  NS_ASSERT(std::isfinite(ctx.stats.sumPhyPayloadBytes));
  NS_ASSERT(std::isfinite(ctx.stats.airtimeFragMsSum));
}

double
PqcExchangeApplication::ComputeFragmentAirtimeMs(uint32_t payloadPhy) const
{
  NS_ASSERT(payloadPhy > 0);

  const double airtimeMs =
      LpwanLoraAirtime::AirtimeMs(m_sf, payloadPhy, m_bwHz, 1);

  return airtimeMs;
}

double
PqcExchangeApplication::GetSensitivityDbm(uint32_t sf,
                                          uint32_t bwHz,
                                          double noiseFigureDb) const
{
  NS_ASSERT(sf >= 7 && sf <= 12);
  NS_ASSERT(bwHz > 0);

  const double noiseFloorDbm =
      -174.0 + 10.0 * std::log10(static_cast<double>(bwHz)) + noiseFigureDb;

  const double snrMinDb = GetSnrMinDb(sf);
  const double sensitivityDbm = noiseFloorDbm + snrMinDb;

  return sensitivityDbm;
}

double
PqcExchangeApplication::ComputeNoiseDbm() const
{
  NS_ASSERT(m_bwHz > 0);

  const double bwHz = static_cast<double>(m_bwHz);
  const double noiseDbm =
      -174.0 + 10.0 * std::log10(bwHz) + m_noiseFigureDb;

  return noiseDbm;
}

double
PqcExchangeApplication::ComputePathLossDb(double dM) const
{
  NS_ASSERT(m_refDistanceM > 0.0);
  NS_ASSERT(m_pathLossExp > 0.0);

  const double d0 = std::max(0.001, m_refDistanceM);
  const double d = std::max(0.001, dM);

  const double basePathLossDb =
      m_refPathLossDb + 10.0 * m_pathLossExp * std::log10(d / d0);

  double shadowingDb = 0.0;
  if (m_shadowingSigmaDb > 0.0 && m_shadowRv)
    {
      shadowingDb = m_shadowingSigmaDb * m_shadowRv->GetValue();
    }

  const double pathLossDb = basePathLossDb + shadowingDb;

  return pathLossDb;
}

double
PqcExchangeApplication::ComputeSnrDb(double dM) const
{
  const double pathLossDb = ComputePathLossDb(dM);
  const double rxDbm = m_txPowerDbm - pathLossDb;
  const double noiseDbm = ComputeNoiseDbm();
  const double snrDb = rxDbm - noiseDbm;

  return snrDb;
}

double
PqcExchangeApplication::ComputeChannelFailureProbability(uint32_t payloadBytes,
                                                         double snrDb)
{
  /*
   * Contract:
   *   Return the nominal PHY/channel decoding failure probability for one
   *   DATA or ACK frame, excluding collisions and RX duty-cycle sleep.
   *
   * Current model:
   *   - delegated to LpwanPacketErrorModel
   *   - clamped to [0, 1]
   *
   * Important:
   *   This function models only PHY/channel failure.
   *   Total transmission failure is composed later as:
   *
   *     p_total = 1 - (1 - p_phy) * (1 - p_collision)
   */
  if (!m_per)
    {
      return 0.0;
    }

  double pPhy = m_per->GetPer(snrDb, m_sf, payloadBytes);
  pPhy = std::clamp(pPhy, 0.0, 1.0);

  return pPhy;
}

double
PqcExchangeApplication::ComputeCollisionFailureProbability(double nowS,
                                                           double txAirtimeMsTotal,
                                                           uint32_t logicalChannels) const
{
  /*
   * Contract:
   *   Return collision-induced failure probability for a transmission
   *   starting at nowS with duration txAirtimeMsTotal.
   *
   * Behavior:
   *   - if no collision model is installed -> 0.0
   *   - otherwise delegate to m_col
   *   - the returned value is guaranteed finite and clamped to [0,1]
   *
   * Important:
   *   This function is a critical observability point for:
   *     - bursty ON/OFF behavior
   *     - lambda(t) variation
   *     - channel occupancy effects
   *
   * Robustness:
   *   - Never propagate NaN or Inf to exchange accumulators.
   */

  NS_ASSERT(nowS >= 0.0);
  NS_ASSERT(txAirtimeMsTotal >= 0.0);

  if (!m_col)
    {
      return 0.0;
    }

  uint32_t logicalChannelsEff = logicalChannels;
  if (logicalChannelsEff == 0)
    {
      logicalChannelsEff = m_col->GetLogicalChannels();
    }

  NS_ASSERT(logicalChannelsEff > 0);

  const double rawPCol =
      m_col->GetCollisionProbabilityAt(nowS,
                                       txAirtimeMsTotal,
                                       logicalChannelsEff);

  if (!std::isfinite(rawPCol))
    {
      NS_LOG_ERROR("COMPUTE_COLLISION_FAILURE_PROBABILITY_INVALID_RAW"
                   << " nowS=" << nowS
                   << " airtimeMs=" << txAirtimeMsTotal
                   << " logicalChannels=" << logicalChannelsEff
                   << " rawPCol=" << rawPCol);
      return 0.0;
    }

  const double pCol = std::clamp(rawPCol, 0.0, 1.0);

  NS_ASSERT(std::isfinite(pCol));
  NS_ASSERT(pCol >= 0.0 && pCol <= 1.0);

  double lambdaEff = 0.0;
  bool isOn = false;

  lambdaEff = m_col->GetEffectiveLambdaTotal(nowS);
  isOn = m_col->GetIsOnAt(nowS);

  if (!std::isfinite(lambdaEff) || lambdaEff < 0.0)
    {
      NS_LOG_ERROR("COMPUTE_COLLISION_FAILURE_PROBABILITY_INVALID_LAMBDA"
                   << " nowS=" << nowS
                   << " airtimeMs=" << txAirtimeMsTotal
                   << " logicalChannels=" << logicalChannelsEff
                   << " lambdaEff=" << lambdaEff);
      lambdaEff = 0.0;
    }

  NS_LOG_INFO("COLLISION_PROB"
              << " nowS=" << nowS
              << " airtimeMs=" << txAirtimeMsTotal
              << " logicalChannels=" << logicalChannelsEff
              << " lambdaEff=" << lambdaEff
              << " isOn=" << (isOn ? 1 : 0)
              << " rawPCol=" << rawPCol
              << " pCol=" << pCol);

  return pCol;
}

double
PqcExchangeApplication::ComputeTotalFailureProbability(double pPhy,
                                                       double pCol) const
{
  /*
   * Contract:
   *   Compute the total per-fragment failure probability combining
   *   physical-layer errors and collision-induced errors.
   *
   * Inputs:
   *   pPhy : probability of PHY failure (e.g., due to SNR / PER model)
   *   pCol : probability of collision-induced failure
   *
   * Behavior:
   *   - both probabilities are clamped to [0, 1]
   *   - total failure probability is computed assuming independence:
   *
   *       pTot = 1 - (1 - pPhy) * (1 - pCol)
   *
   * Interpretation:
   *   - (1 - pPhy) is the probability of successful PHY reception
   *   - (1 - pCol) is the probability of no collision
   *   - their product is the probability of successful transmission
   *   - 1 - (...) gives total failure probability
   *
   * Important:
   *   - This formulation assumes statistical independence between PHY errors
   *     and collisions.
   *   - This is a modeling assumption consistent with standard ALOHA analyses.
   *
   * Safety:
   *   - output is guaranteed to be in [0, 1]
   */

  pPhy = std::clamp(pPhy, 0.0, 1.0);
  pCol = std::clamp(pCol, 0.0, 1.0);

  const double pTot = 1.0 - (1.0 - pPhy) * (1.0 - pCol);

  return std::clamp(pTot, 0.0, 1.0);
}

void
PqcExchangeApplication::FinalizeExchange(ExchangeContext& ctx) const
{
  /*
   * Contract:
   *   Finalize one fully executed exchange by converting internal runtime,
   *   outcome, and statistics state into the exported ExchangeResult.
   *
   * Responsibilities:
   *   - publish final fragment-level exchange outcomes
   *   - compute exchange-level observed PER and averaged diagnostics
   *   - compute airtime, latency, and energy summaries
   *   - publish DATA / ACK frame accounting using real transmitted attempts
   *   - publish ACK and RX-sleep diagnostics
   *   - enforce final consistency invariants before CSV export
   *
   * Important:
   *   - This function is the final authority that materializes ExchangeResult.
   *   - It must not change protocol semantics; it only converts already
   *     accumulated runtime state into exported metrics.
   *   - All averages normalized by fragment count use the realized exchange
   *     plan size ctx.runtime.nFrags.
   */

  const double invN =
      (ctx.runtime.nFrags > 0)
          ? (1.0 / static_cast<double>(ctx.runtime.nFrags))
          : 0.0;

  // --------------------------------------------------------------------------
  // Final logical exchange outcomes
  // --------------------------------------------------------------------------
  ctx.out.okFrags = ctx.outcome.okFrags;
  ctx.out.failFrags = ctx.outcome.failFrags;
  ctx.out.perExchangeFail = ctx.outcome.exchangeFailed;
  ctx.out.retransmissionsCount = ctx.outcome.totalRetx;

  /*
   * Observed exchange-level fragment error rate:
   *
   *   failFrags / nFrags
   *
   * This is an exchange-level metric over logical fragments, not a per-attempt
   * or per-frame quantity.
   */
  ctx.out.perObserved =
      (ctx.runtime.nFrags > 0)
          ? (static_cast<double>(ctx.outcome.failFrags) /
             static_cast<double>(ctx.runtime.nFrags))
          : 0.0;

  // --------------------------------------------------------------------------
  // Fragment-normalized averages and nominal diagnostics
  // --------------------------------------------------------------------------
  ctx.out.phyPayloadBytesFragAvg = ctx.stats.sumPhyPayloadBytes * invN;
  ctx.out.perChannelAvg = ctx.stats.sumPhy * invN;
  ctx.out.perCollisionSimAvg = ctx.stats.sumColSim * invN;
  ctx.out.perCollisionThAvg = ctx.stats.sumColTh * invN;
  ctx.out.perTotalAvg = ctx.stats.sumTot * invN;
  ctx.out.perFragmentAvg = ctx.out.perTotalAvg;
  ctx.out.airtimeMsFragAvg = ctx.stats.airtimeFragMsSum * invN;

  // --------------------------------------------------------------------------
  // Airtime and latency
  // --------------------------------------------------------------------------
  ctx.out.txAirtimeMsTotal = ctx.stats.txAirtimeMsSum;
  ctx.out.exchangeLatencyS = ctx.stats.elapsedMsSum / 1000.0;

  // --------------------------------------------------------------------------
  // Bursty / phase-aware attempt diagnostics
  // --------------------------------------------------------------------------
  ctx.out.attemptsTotal = ctx.stats.attemptsTotal;
  ctx.out.attemptsOn = ctx.stats.attemptsOn;
  ctx.out.attemptsOff = ctx.stats.attemptsOff;

  if (ctx.stats.attemptsTotal > 0)
    {
      ctx.out.lambdaEffAttemptAvg =
          ctx.stats.lambdaEffSum /
          static_cast<double>(ctx.stats.attemptsTotal);

      ctx.out.attemptsOffFraction =
          static_cast<double>(ctx.stats.attemptsOff) /
          static_cast<double>(ctx.stats.attemptsTotal);
    }
  else
    {
      ctx.out.lambdaEffAttemptAvg = 0.0;
      ctx.out.attemptsOffFraction = 0.0;
    }

  /*
   * nFragmentsExchange belongs to the realized exchange plan and is finalized
   * here after execution.
   */
  ctx.out.nFragmentsExchange = ctx.runtime.nFrags;

  // --------------------------------------------------------------------------
  // Energy model
  // --------------------------------------------------------------------------
  double rxListenMs = 0.0;

  if (m_rxDutyCycleEnabled)
    {
      /*
       * Under RX duty cycling, RX listening time is modeled as the active
       * fraction of the total exchange elapsed time.
       */
      const double periodMs = m_rxWindowMs + m_rxIdleMs;
      const double activeFraction =
          (periodMs > 0.0) ? (m_rxWindowMs / periodMs) : 1.0;

      ctx.stats.rxActiveMsReal = ctx.stats.elapsedMsSum * activeFraction;
      rxListenMs = ctx.stats.rxActiveMsReal;
    }
  else
    {
      /*
       * Without RX duty cycling, RX energy is modeled either from total
       * exchange elapsed time or from TX airtime only, depending on the
       * configured energy policy.
       */
      rxListenMs =
          m_rxEnergyUsesExchangeLatency
              ? ctx.stats.elapsedMsSum
              : ctx.stats.txAirtimeMsSum;
    }

  ctx.out.airtimeEnergyTx = m_txEnergyJPerMs * ctx.out.txAirtimeMsTotal;
  ctx.out.airtimeEnergyRx = m_rxEnergyJPerMs * rxListenMs;

  ctx.out.energyJExchangeTx = ctx.out.airtimeEnergyTx;
  ctx.out.energyJExchangeRx = ctx.out.airtimeEnergyRx;
  ctx.out.energyJExchange = ctx.out.energyJExchangeTx + ctx.out.energyJExchangeRx;

  ctx.out.energyPerFragmentTx =
      (ctx.runtime.nFrags > 0)
          ? (ctx.out.energyJExchangeTx /
             static_cast<double>(ctx.runtime.nFrags))
          : 0.0;

  ctx.out.energyPerFragmentRx =
      (ctx.runtime.nFrags > 0)
          ? (ctx.out.energyJExchangeRx /
             static_cast<double>(ctx.runtime.nFrags))
          : 0.0;

  // --------------------------------------------------------------------------
  // DATA / ACK frame accounting
  // --------------------------------------------------------------------------
  /*
   * DATA frame accounting must reflect real transmitted DATA attempts,
   * not logical fragments planned for the exchange.
   *
   * Why:
   *   - an exchange may abort early after one fragment exhausts retries
   *   - remaining logical fragments may never be attempted
   *   - therefore nFrags + totalRetx over-counts transmitted DATA frames
   */
  ctx.out.dataFramesTx = ctx.outcome.dataFramesTxReal;
  ctx.out.ackFramesTx = ctx.ack.ackFramesTx;
  ctx.out.framesTxTotal = ctx.out.dataFramesTx + ctx.out.ackFramesTx;

  // --------------------------------------------------------------------------
  // RX-sleep diagnostics
  // --------------------------------------------------------------------------
  ctx.out.lostRxSleepAck = ctx.stats.lostRxSleepAck;
  ctx.out.lostRxSleepData = ctx.stats.lostRxSleepData;

  // --------------------------------------------------------------------------
  // ACK diagnostics
  // --------------------------------------------------------------------------
  ctx.out.ackAirtimeMs = ctx.ack.ackAirtimeMsSum;
  ctx.out.ackLogicalFailures = ctx.ack.ackLogicalFailures;
  ctx.out.lostAckSleep = ctx.ack.lostAckSleep;
  ctx.out.lostAckChannel = ctx.ack.lostAckChannel;
  ctx.out.ackBatchRetxRounds = ctx.ack.ackBatchRetxRounds;

  if (ctx.ack.ackAttemptsTotal > 0)
    {
      ctx.out.ackPerChannelAvg =
          ctx.ack.ackPerChannelSum /
          static_cast<double>(ctx.ack.ackAttemptsTotal);

      ctx.out.ackPerCollisionAvg =
          ctx.ack.ackPerCollisionSum /
          static_cast<double>(ctx.ack.ackAttemptsTotal);

      ctx.out.ackPerTotalAvg =
          ctx.ack.ackPerTotalSum /
          static_cast<double>(ctx.ack.ackAttemptsTotal);
    }
  else
    {
      ctx.out.ackPerChannelAvg = 0.0;
      ctx.out.ackPerCollisionAvg = 0.0;
      ctx.out.ackPerTotalAvg = 0.0;
    }

  // --------------------------------------------------------------------------
  // Final diagnostics and consistency checks
  // --------------------------------------------------------------------------
  NS_LOG_INFO("FINAL_DATA_TX"
              << " real=" << ctx.outcome.dataFramesTxReal
              << " totalRetx=" << ctx.outcome.totalRetx
              << " nFrags=" << ctx.runtime.nFrags);

  NS_ASSERT(ctx.out.framesTxTotal ==
            (ctx.out.dataFramesTx + ctx.out.ackFramesTx));
  NS_ASSERT(ctx.out.dataFramesTx >= ctx.out.okFrags);
  NS_ASSERT(ctx.out.txAirtimeMsTotal >= 0.0);
  NS_ASSERT(ctx.out.exchangeLatencyS >= 0.0);
  NS_ASSERT(ctx.out.perObserved >= 0.0 && ctx.out.perObserved <= 1.0);
  NS_ASSERT(ctx.out.perChannelAvg >= 0.0 && ctx.out.perChannelAvg <= 1.0);
  NS_ASSERT(ctx.out.perCollisionSimAvg >= 0.0 &&
            ctx.out.perCollisionSimAvg <= 1.0);
  NS_ASSERT(ctx.out.perTotalAvg >= 0.0 && ctx.out.perTotalAvg <= 1.0);
  NS_ASSERT(ctx.out.ackPerChannelAvg >= 0.0 &&
            ctx.out.ackPerChannelAvg <= 1.0);
  NS_ASSERT(ctx.out.ackPerCollisionAvg >= 0.0 &&
            ctx.out.ackPerCollisionAvg <= 1.0);
  NS_ASSERT(ctx.out.ackPerTotalAvg >= 0.0 &&
            ctx.out.ackPerTotalAvg <= 1.0);

  NS_LOG_INFO("FINALIZE_EXCHANGE_OUT"
              << " perObserved=" << ctx.out.perObserved
              << " perChannelAvg=" << ctx.out.perChannelAvg
              << " perCollisionSimAvg=" << ctx.out.perCollisionSimAvg
              << " perCollisionThAvg=" << ctx.out.perCollisionThAvg
              << " perTotalAvg=" << ctx.out.perTotalAvg
              << " txAirtimeMsTotal=" << ctx.out.txAirtimeMsTotal
              << " exchangeLatencyS=" << ctx.out.exchangeLatencyS
              << " dataFramesTx=" << ctx.out.dataFramesTx
              << " ackFramesTx=" << ctx.out.ackFramesTx
              << " framesTxTotal=" << ctx.out.framesTxTotal
              << " lostRxSleepData=" << ctx.out.lostRxSleepData
              << " lostRxSleepAck=" << ctx.out.lostRxSleepAck
              << " lostAckSleep=" << ctx.out.lostAckSleep
              << " lostAckChannel=" << ctx.out.lostAckChannel
              << " ackLogicalFailures=" << ctx.out.ackLogicalFailures
              << " ackBatchRetxRounds=" << ctx.out.ackBatchRetxRounds
              << " perExchangeFail=" << ctx.out.perExchangeFail
              << " retransmissionsCount=" << ctx.out.retransmissionsCount);
}

void
PqcExchangeApplication::AccumulateAttemptDiagnostics(ExchangeContext& ctx,
                                                     double attemptNowS) const
{
  /*
   * Contract:
   *   Accumulate attempt-level burst/load diagnostics at the actual timestamp
   *   of a real DATA transmission attempt.
   *
   * Responsibilities:
   *   - count one attempt-level diagnostic sample
   *   - snapshot whether the bursty interferer process is ON or OFF
   *   - accumulate the effective total interferer attempt rate at this instant
   *
   * Important:
   *   This method does NOT:
   *     - decide success or failure
   *     - compute collision probability
   *     - modify fragment outcome counters
   *
   * It is purely a diagnostic accumulator for exchange-level exported metrics.
   */

  NS_ASSERT(attemptNowS >= 0.0);

  if (!m_col)
    {
      return;
    }

  const bool isOn = m_col->GetIsOnAt(attemptNowS);
  const double lambdaEff =
      std::max(0.0, m_col->GetEffectiveLambdaTotal(attemptNowS));

  ctx.stats.attemptsTotal++;
  ctx.stats.lambdaEffSum += lambdaEff;

  if (isOn)
    {
      ctx.stats.attemptsOn++;
    }
  else
    {
      ctx.stats.attemptsOff++;
    }
}

void
PqcExchangeApplication::BuildFragmentPlan(std::vector<uint32_t>& payloadBytesPerFragment) const
{
  /*
   * Contract:
   *   Build the full logical fragmentation plan for one PQC exchange.
   *
   * Behavior:
   *   - if AutoFragments=false:
   *       build exactly (PkFragments + CtFragments) fragments, each with
   *       configured payload size PayloadBytes
   *
   *   - if AutoFragments=true:
   *       derive the effective crypto payload size from:
   *         * configured PayloadBytes
   *         * PHY payload cap for the current SF (or MaxPayloadOverrideBytes)
   *         * G2G overhead
   *       then fragment PK and CT byte streams separately and concatenate them
   *       in exchange order: [PK fragments][CT fragments]
   *
   * Important:
   *   - this function defines the real per-fragment logical payload plan used
   *     by the exchange
   *   - fragment payload sizes here are crypto/application payload sizes,
   *     not full PHY payload sizes
   */

  payloadBytesPerFragment.clear();

  if (!m_autoFragments)
    {
      /*
       * Manual fragmentation mode:
       * the exchange is defined directly by fragment counts and configured
       * payload size.
       */
      NS_ABORT_MSG_IF(m_payloadBytes == 0,
                      "BuildFragmentPlan: PayloadBytes must be > 0 when AutoFragments=false.");

      const uint32_t totalFragments = m_pkFragments + m_ctFragments;

      payloadBytesPerFragment.resize(totalFragments, m_payloadBytes);

      NS_LOG_INFO("BUILD_FRAGMENT_PLAN_MANUAL"
                  << " pkFragments=" << m_pkFragments
                  << " ctFragments=" << m_ctFragments
                  << " totalFragments=" << totalFragments
                  << " payloadBytes=" << m_payloadBytes);

      return;
    }

  /*
   * Auto-fragmentation mode:
   * compute the effective crypto payload cap after accounting for PHY capacity
   * and protocol overhead.
   */
  uint32_t capPhy =
      (m_maxPayloadOverrideBytes > 0)
          ? m_maxPayloadOverrideBytes
          : LpwanLoraAirtime::MaxPayloadBytes(m_sf);

  capPhy = std::max<uint32_t>(1, capPhy);

  NS_ABORT_MSG_IF(m_payloadBytes == 0,
                  "BuildFragmentPlan: PayloadBytes must be > 0 when AutoFragments=true.");

  NS_ABORT_MSG_IF(capPhy <= m_g2gOverheadBytes,
                  "BuildFragmentPlan: PHY payload cap must exceed G2gOverheadBytes so that at least 1 crypto byte fits.");

  const uint32_t maxCryptoPayload = capPhy - m_g2gOverheadBytes;

  uint32_t effectivePayloadBytes = m_payloadBytes;

  if (effectivePayloadBytes > maxCryptoPayload)
    {
      NS_LOG_WARN("BUILD_FRAGMENT_PLAN_AUTO_CLAMP"
                  << " sf=" << static_cast<uint32_t>(m_sf)
                  << " payloadBytesConfig=" << m_payloadBytes
                  << " g2gOverheadBytes=" << m_g2gOverheadBytes
                  << " capPhy=" << capPhy
                  << " maxCryptoPayload=" << maxCryptoPayload
                  << " effectivePayloadBytes=" << maxCryptoPayload);

      effectivePayloadBytes = maxCryptoPayload;
    }

  std::vector<uint32_t> pkFrags;
  std::vector<uint32_t> ctFrags;

  BuildFragmentPlan(pkFrags, m_pkBytes, effectivePayloadBytes);
  BuildFragmentPlan(ctFrags, m_ctBytes, effectivePayloadBytes);

  payloadBytesPerFragment.reserve(pkFrags.size() + ctFrags.size());
  payloadBytesPerFragment.insert(payloadBytesPerFragment.end(),
                                 pkFrags.begin(),
                                 pkFrags.end());
  payloadBytesPerFragment.insert(payloadBytesPerFragment.end(),
                                 ctFrags.begin(),
                                 ctFrags.end());

  NS_LOG_INFO("BUILD_FRAGMENT_PLAN_AUTO"
              << " pkBytes=" << m_pkBytes
              << " ctBytes=" << m_ctBytes
              << " payloadBytesConfig=" << m_payloadBytes
              << " effectivePayloadBytes=" << effectivePayloadBytes
              << " pkFragments=" << pkFrags.size()
              << " ctFragments=" << ctFrags.size()
              << " totalFragments=" << payloadBytesPerFragment.size());
}

void
PqcExchangeApplication::BuildFragmentPlan(std::vector<uint32_t>& fragPayloads,
                                          uint32_t totalBytes,
                                          uint32_t payloadBytes) const
{
  /*
   * Contract:
   *   Build a deterministic fragmentation plan for a payload of totalBytes
   *   using a maximum per-fragment payload size payloadBytes.
   *
   * Behavior:
   *   - Each fragment carries at most payloadBytes bytes
   *   - The last fragment may be smaller
   *   - The sum of all fragment payloads equals totalBytes
   *
   * Important:
   *   - payloadBytes MUST be > 0 (configuration error otherwise)
   *   - fragmentation is purely logical (no PHY header here)
   *
   * Rationale:
   *   This function defines the fragmentation granularity that directly
   *   impacts airtime, PER, and energy accounting.
   */

  fragPayloads.clear();

  /*
   * Degenerate case: empty payload → no fragments.
   */
  if (totalBytes == 0)
    {
      NS_LOG_INFO("BUILD_FRAGMENT_PLAN"
                  << " totalBytes=0"
                  << " nFragments=0");
      return;
    }

  /*
   * Configuration validation.
   */
  NS_ABORT_MSG_IF(payloadBytes == 0,
                  "BuildFragmentPlan: payloadBytes must be > 0 "
                  "(invalid fragmentation configuration)");

  const uint32_t fragSize = payloadBytes;

  uint32_t remaining = totalBytes;

  while (remaining > 0)
    {
      const uint32_t thisFrag = std::min(remaining, fragSize);
      fragPayloads.push_back(thisFrag);
      remaining -= thisFrag;
    }

  /*
   * Strong consistency check.
   */
  uint64_t sum = 0;
  for (uint32_t v : fragPayloads)
    {
      sum += v;
    }

  NS_ASSERT_MSG(sum == totalBytes,
                "BuildFragmentPlan: fragment sum mismatch");

  /*
   * Diagnostic logging (useful for reproducibility).
   */
  const uint32_t nFrags = static_cast<uint32_t>(fragPayloads.size());
  const uint32_t lastFrag = fragPayloads.back();

  NS_LOG_INFO("BUILD_FRAGMENT_PLAN"
              << " totalBytes=" << totalBytes
              << " payloadBytes=" << payloadBytes
              << " nFragments=" << nFrags
              << " lastFragmentBytes=" << lastFrag);
}

void
PqcExchangeApplication::ApplyAckRecoveryGap(ExchangeContext& ctx,
                                            PendingAckEntry& e)
{
  /*
   * Contract:
   *   Apply ACK-recovery pacing before redelivering one DATA fragment after
   *   an ACK failure.
   *
   * Inputs:
   *   ctx : exchange runtime/state container
   *   e   : fragment state for the fragment that will be redelivered
   *
   * Responsibilities:
   *   - build the ACK-recovery gap context from the current application
   *     configuration
   *   - compute the ACK-recovery gap using the active pacing policy
   *   - advance exchange runtime time by the computed gap
   *   - accumulate exchange elapsed-time accounting for that gap
   *   - emit a diagnostic log entry for reproducibility and debugging
   *
   * Important:
   *   - This function does NOT redeliver the DATA fragment itself.
   *     The caller must invoke RunFragmentArq(...) after this gap is applied.
   *
   *   - This function does NOT consume retry budget.
   *     DATA retry consumption remains the responsibility of RunFragmentArq().
   *
   *   - The applied recovery gap contributes to exchange latency
   *     (ctx.stats.elapsedMsSum), but it is not TX airtime.
   *     Therefore:
   *       * ctx.stats.elapsedMsSum is incremented
   *       * ctx.stats.txAirtimeMsSum is NOT incremented
   *
   *   - recoveryRound is taken from ctx.ack.ackBatchRetxRounds and is expected
   *     to already reflect the current ACK recovery round being scheduled.
   *
   * Timing semantics:
   *   If the current runtime time is t_now and the computed gap is gapMs,
   *   the new runtime time becomes:
   *
   *       t_now + gapMs / 1000
   */

  AckRecoveryGapContext gapCtx;
  gapCtx.sf = m_sf;
  gapCtx.bandwidthHz = static_cast<double>(m_bwHz);
  gapCtx.ackRecoveryMarginDb = m_ackRecoveryMarginDb;
  gapCtx.recoveryRound = ctx.ack.ackBatchRetxRounds;
  gapCtx.pacingMode = m_pacing;
  gapCtx.fixedGapMs = static_cast<double>(m_fixedGapMs);
  gapCtx.gapMinMs = static_cast<double>(m_gapMinMs);
  gapCtx.gapMaxMs = static_cast<double>(m_gapMaxMs);
  gapCtx.kTsym = m_kTsym;
  gapCtx.sfAwarePacingThreshold = m_sfAwarePacingThreshold;
  gapCtx.ackRecoveryMarginRefDb = m_ackRecoveryMarginRefDb;
  gapCtx.ackRecoveryAlpha = m_ackRecoveryAlpha;
  gapCtx.ackRecoveryBackoffGamma = m_ackRecoveryBackoffGamma;
  gapCtx.rxDutyCycleEnabled = m_rxDutyCycleEnabled;
  gapCtx.rxWindowMs = m_rxWindowMs;
  gapCtx.rxIdleMs = m_rxIdleMs;

  const double gapMs = ComputeAckRecoveryGapMs(gapCtx);

  const double nowBeforeS = ctx.runtime.nowS;
  ctx.runtime.nowS += gapMs / 1000.0;
  ctx.stats.elapsedMsSum += gapMs;

  NS_LOG_INFO("APPLY_ACK_RECOVERY_GAP"
              << " fragIndex=" << e.fragIndex
              << " retxUsed=" << e.retxUsed
              << " ackBatchRetxRounds=" << ctx.ack.ackBatchRetxRounds
              << " gapMs=" << gapMs
              << " nowBeforeS=" << nowBeforeS
              << " nowAfterS=" << ctx.runtime.nowS);
}

bool
PqcExchangeApplication::FlushAckBatch(ExchangeContext& ctx, bool force)
{
  /*
   * Contract:
   *   Attempt logical acknowledgment of all fragments currently pending in the
   *   ACK batch queue.
   *
   * Behavior:
   *   - if force == false and batch threshold is not reached, do nothing
   *   - otherwise transmit one ACK frame for the whole pending batch
   *   - if ACK succeeds, all pending fragments become exchange-level successes
   *   - if ACK fails, enter ACK recovery:
   *       * previously DATA-delivered fragments are redelivered
   *       * if recovery succeeds, fragments remain pending and a new ACK round
   *         is attempted
   *       * if any redelivery fails terminally, the exchange fails
   *
   * Important:
   *   This method is the single authority for promoting DATA-delivered
   *   fragments into exchange-level successful fragments:
   *
   *     ctx.outcome.okFrags += batchSize
   *
   *   Terminal failFrags caused by redelivery failure are counted inside
   *   RunFragmentArq(). This method must not duplicate that accounting.
   *
   * Robustness fix:
   *   On ACK-recovery failure, all fragments that are still logically pending
   *   must remain recoverable by RunOneExchange() final reconciliation.
   *   Therefore, this function must never lose ownership of not-yet-processed
   *   pending fragments during partial ACK recovery.
   */

  if (ctx.ack.pendingAckQueue.empty())
    {
      return true;
    }

  if (!force && ctx.ack.pendingAckQueue.size() < m_ackBatchSize)
    {
      return true;
    }

  while (!ctx.ack.pendingAckQueue.empty())
    {
      const uint32_t batchSize =
          static_cast<uint32_t>(ctx.ack.pendingAckQueue.size());

      const uint32_t ackPayloadBytes = LORAWAN_ACK_PHY_BYTES;
      const double ackAirtimeMs = ComputeFragmentAirtimeMs(ackPayloadBytes);

      double ackNowS = ctx.runtime.nowS;

      /*
       * In phase-bounded mode under RX duty cycling, align ACK transmission
       * to the next RX-active instant that can fully contain the ACK frame.
       */
      if (m_rxDutyCycleEnabled &&
            (m_pacing == PacingMode::RADIOAWARE_PHASE_BOUNDED ||
             m_pacing == PacingMode::STOCHASTIC))
        {
          ackNowS = AlignToRxWindowStartForFullContainment(ackNowS,
                                                           ackAirtimeMs,
                                                           m_rxWindowMs,
                                                           m_rxIdleMs);
          ctx.runtime.nowS = ackNowS;
        }

      const double ackEndS = ackNowS + ackAirtimeMs / 1000.0;

      /*
       * Every ACK transmission is a real on-air frame.
       */
      ctx.ack.ackFramesTx++;
      ctx.ack.ackAirtimeMsSum += ackAirtimeMs;
      ctx.stats.txAirtimeMsSum += ackAirtimeMs;
      ctx.stats.elapsedMsSum += ackAirtimeMs;

      bool ackSeen = true;
      bool lostBySleep = false;
      bool lostByChannel = false;

      const double ackPPhy =
          ComputeChannelFailureProbability(ackPayloadBytes, ctx.out.snrDb);

      const double ackPCol =
          ComputeCollisionFailureProbability(ackNowS,
                                             ackAirtimeMs,
                                             ctx.runtime.logicalChannels);

      const double ackPTot =
          ComputeTotalFailureProbability(ackPPhy, ackPCol);

      ctx.ack.ackPerChannelSum += ackPPhy;
      ctx.ack.ackPerCollisionSum += ackPCol;
      ctx.ack.ackPerTotalSum += ackPTot;
      ctx.ack.ackAttemptsTotal++;

      NS_LOG_INFO("ACK_ATTEMPT_BEGIN"
                  << " batchSize=" << batchSize
                  << " ackNowS=" << ackNowS
                  << " ackEndS=" << ackEndS
                  << " ackAirtimeMs=" << ackAirtimeMs
                  << " ackPPhy=" << ackPPhy
                  << " ackPCol=" << ackPCol
                  << " ackPTot=" << ackPTot);

      /*
       * Deterministic RX sleep gating for ACK reception.
       */
      if (m_rxDutyCycleEnabled)
        {
          ackSeen = IsFrameFullyContainedInRxWindow(ackNowS * 1000.0,
                                                    ackAirtimeMs,
                                                    m_rxWindowMs,
                                                    m_rxIdleMs);
          if (!ackSeen)
            {
              lostBySleep = true;
            }
        }

      /*
       * Stochastic PHY/collision failure for ACK.
       */
      if (ackSeen && (m_uv->GetValue() < ackPTot))
        {
          ackSeen = false;
          lostByChannel = true;
        }

      ctx.runtime.nowS = ackEndS;

      /*
       * ACK success:
       * all pending fragments become exchange-level successes.
       */
      if (ackSeen)
        {
          ctx.outcome.okFrags += batchSize;

          NS_LOG_INFO("ACK_ATTEMPT_RESULT"
                      << " result=success"
                      << " batchSize=" << batchSize
                      << " okFrags=" << ctx.outcome.okFrags
                      << " ackFramesTx=" << ctx.ack.ackFramesTx);

          ctx.ack.pendingAckQueue.clear();
          return true;
        }

      /*
       * ACK failed: update diagnostics and enter ACK recovery.
       */
      if (lostBySleep)
        {
          ctx.ack.lostAckSleep++;
          ctx.stats.lostRxSleepAck++;
        }

      if (lostByChannel)
        {
          ctx.ack.lostAckChannel++;
        }

      NS_LOG_INFO("ACK_ATTEMPT_RESULT"
                  << " result=fail"
                  << " batchSize=" << batchSize
                  << " cause=" << (lostBySleep ? "rx_sleep"
                                               : "channel_or_collision")
                  << " lostAckSleep=" << ctx.ack.lostAckSleep
                  << " lostAckChannel=" << ctx.ack.lostAckChannel);

      ctx.ack.ackBatchRetxRounds++;
      ctx.ack.ackLogicalFailures += batchSize;

      /*
       * All currently pending fragments must be redelivered before retrying ACK.
       *
       * Robustness rule:
       *   Never lose the logical ownership of still-pending fragments.
       *   We therefore:
       *     - snapshot the old batch,
       *     - rebuild a new pending queue as redeliveries succeed,
       *     - if recovery aborts mid-batch, restore all not-yet-classified
       *       fragments into pendingAckQueue so RunOneExchange() can reconcile
       *       them at exchange level.
       */
      const std::vector<PendingAckEntry> oldPending = ctx.ack.pendingAckQueue;
      std::vector<PendingAckEntry> rebuiltPending;
      rebuiltPending.reserve(oldPending.size());

      ctx.ack.pendingAckQueue.clear();

      for (size_t idx = 0; idx < oldPending.size(); ++idx)
        {
          PendingAckEntry e = oldPending[idx];

          NS_LOG_INFO("ACK_RECOVERY_ITEM_BEGIN"
                      << " idx=" << idx
                      << " fragIndex=" << e.fragIndex
                      << " retxUsed=" << e.retxUsed
                      << " rebuiltPendingSize=" << rebuiltPending.size()
                      << " oldPendingSize=" << oldPending.size());

          /*
           * If the DATA retry budget is already exhausted, the exchange fails.
           *
           * Do not increment failFrags here: RunOneExchange() will reconcile
           * pending/not-started fragments at exchange level, and terminal DATA
           * fragment failures are counted inside RunFragmentArq().
           *
           * Crucially, preserve the current fragment and all not-yet-processed
           * remaining fragments as logically pending so they are not lost.
           */
          if (e.retxUsed >= m_maxRetries)
            {
              rebuiltPending.insert(rebuiltPending.end(),
                                    oldPending.begin() + static_cast<std::ptrdiff_t>(idx),
                                    oldPending.end());
              ctx.ack.pendingAckQueue = std::move(rebuiltPending);

              NS_LOG_INFO("ACK_RECOVERY_ABORT"
                          << " reason=retry_budget_exhausted"
                          << " idx=" << idx
                          << " fragIndex=" << e.fragIndex
                          << " restoredPending=" << ctx.ack.pendingAckQueue.size());

              ctx.outcome.exchangeFailed = true;
              return false;
            }

          /*
           * Apply ACK recovery pacing before redelivering the DATA fragment.
           */
          ApplyAckRecoveryGap(ctx, e);

          /*
           * Redeliver the DATA fragment.
           *
           * RunFragmentArq() owns DATA retry consumption and terminal DATA
           * failure accounting.
           */
          const bool redelivered = RunFragmentArq(ctx, e);
          if (!redelivered)
            {
              /*
               * The current fragment has already been accounted for by
               * RunFragmentArq() as a terminal DATA failure. Preserve only the
               * previously rebuilt successes plus the not-yet-processed suffix.
               */
              if (idx + 1 < oldPending.size())
                {
                  rebuiltPending.insert(rebuiltPending.end(),
                                        oldPending.begin() +
                                            static_cast<std::ptrdiff_t>(idx + 1),
                                        oldPending.end());
                }

              ctx.ack.pendingAckQueue = std::move(rebuiltPending);

              NS_LOG_INFO("ACK_RECOVERY_ABORT"
                          << " reason=redelivery_failed"
                          << " idx=" << idx
                          << " fragIndex=" << e.fragIndex
                          << " restoredPending=" << ctx.ack.pendingAckQueue.size());

              ctx.outcome.exchangeFailed = true;
              return false;
            }

          /*
           * DATA redelivery succeeded; fragment remains pending logical ACK.
           */
          rebuiltPending.push_back(e);

          NS_LOG_INFO("ACK_RECOVERY_ITEM_SUCCESS"
                      << " idx=" << idx
                      << " fragIndex=" << e.fragIndex
                      << " rebuiltPendingSize=" << rebuiltPending.size());
        }

      /*
       * Entire ACK recovery round succeeded. Replace the pending queue with the
       * rebuilt set of redelivered fragments and continue the outer loop, which
       * will attempt a new ACK round.
       */
      ctx.ack.pendingAckQueue = std::move(rebuiltPending);

      NS_LOG_INFO("ACK_RECOVERY_ROUND_COMPLETE"
                  << " pendingAfterRecovery=" << ctx.ack.pendingAckQueue.size()
                  << " ackBatchRetxRounds=" << ctx.ack.ackBatchRetxRounds);
    }

  return true;
}

double
PqcExchangeApplication::AlignToRxWindowStartForFullContainment(double nowS,
                                                               double durationMs,
                                                               double rxWindowMs,
                                                               double rxIdleMs) const
{
  /*
   * Contract:
   *   Return the earliest timestamp t >= nowS such that the full interval
   *
   *     [t, t + durationMs]
   *
   *   is fully contained inside one active RX window.
   *
   * Units:
   *   - nowS is in seconds
   *   - durationMs, rxWindowMs, rxIdleMs are in milliseconds
   *
   * Important:
   *   - if RX is effectively always on, return nowS
   *   - if no active RX window exists, no valid alignment is possible
   *   - if the frame duration exceeds the RX window length, no valid
   *     alignment is possible
   */

  NS_ASSERT(nowS >= 0.0);
  NS_ASSERT(durationMs >= 0.0);
  NS_ASSERT(rxWindowMs >= 0.0);
  NS_ASSERT(rxIdleMs >= 0.0);

  /*
   * Always-on receiver: every timestamp is valid.
   */
  if (rxIdleMs <= 0.0)
    {
      return nowS;
    }

  /*
   * No active RX window exists: alignment is impossible.
   */
  NS_ABORT_MSG_IF(rxWindowMs <= 0.0,
                  "Cannot align transmission to RX window: rxWindowMs <= 0.");

  /*
   * A frame longer than the RX window can never fit fully inside any active
   * RX interval.
   */
  NS_ABORT_MSG_IF(durationMs > rxWindowMs,
                  "Cannot align transmission to RX window: frame duration "
                  "exceeds rxWindowMs.");

  const double cycleMs = rxWindowMs + rxIdleMs;

  /*
   * Defensive guard. In practice, rxIdleMs > 0 and rxWindowMs > 0 already
   * imply cycleMs > 0.
   */
  NS_ABORT_MSG_IF(cycleMs <= 0.0,
                  "Invalid RX duty-cycle configuration: rxWindowMs + rxIdleMs <= 0.");

  const double startMs = nowS * 1000.0;

  /*
   * If the current instant already satisfies full-window containment,
   * keep it unchanged.
   */
  if (IsFrameFullyContainedInRxWindow(startMs,
                                      durationMs,
                                      rxWindowMs,
                                      rxIdleMs))
    {
      return nowS;
    }

  /*
   * Otherwise, move to the next RX-window start.
   *
   * RX windows are:
   *
   *   [k * cycleMs, k * cycleMs + rxWindowMs)
   *
   * Therefore, if the current timestamp does not fit in the current cycle,
   * the earliest candidate that can fit a full frame is the start of the next
   * cycle.
   */
  const double cycleIndex = std::floor(startMs / cycleMs);
  const double nextRxStartMs = (cycleIndex + 1.0) * cycleMs;

  /*
   * By construction, nextRxStartMs is the start of an active RX window.
   * Since durationMs <= rxWindowMs was already enforced above, the full frame
   * will fit inside that window.
   */
  return nextRxStartMs / 1000.0;
}

bool
PqcExchangeApplication::IsFrameFullyContainedInRxWindow(double startMs,
                                                        double durationMs,
                                                        double rxWindowMs,
                                                        double rxIdleMs) const
{
  /*
   * Contract:
   *   Return true iff the full frame interval
   *
   *     [startMs, startMs + durationMs]
   *
   *   is fully contained inside one active RX window.
   *
   * Partial overlap does NOT count as successful reception.
   */

  NS_ASSERT(durationMs >= 0.0);
  NS_ASSERT(rxWindowMs >= 0.0);
  NS_ASSERT(rxIdleMs >= 0.0);

  const double periodMs = rxWindowMs + rxIdleMs;

  /*
   * No effective duty cycle: receiver is always available.
   */
  if (periodMs <= 0.0)
    {
      return true;
    }

  /*
   * No active RX window: nothing can be received.
   */
  if (rxWindowMs <= 0.0)
    {
      return false;
    }

  const double endMs = startMs + durationMs;

  /*
   * Zero-duration frame: treat it as contained if its instant falls inside
   * an active RX window.
   */
  if (durationMs <= 0.0)
    {
      double phase = std::fmod(startMs, periodMs);
      if (phase < 0.0)
        {
          phase += periodMs;
        }
      return phase < rxWindowMs;
    }

  /*
   * A frame longer than the RX window can never fit fully.
   */
  if (durationMs > rxWindowMs)
    {
      return false;
    }

  /*
   * Check the current cycle and, defensively, the next one.
   */
  const double baseK = std::floor(startMs / periodMs);

  for (int dk = 0; dk <= 1; ++dk)
    {
      const double winStart = (baseK + static_cast<double>(dk)) * periodMs;
      const double winEnd = winStart + rxWindowMs;

      const bool fullyInside =
          (startMs >= winStart) &&
          (endMs <= winEnd);

      if (fullyInside)
        {
          return true;
        }
    }

  return false;
}

double
PqcExchangeApplication::ComputeFragmentGapMs(const FragmentGapContext& ctx) const
{
  NS_ASSERT(ctx.bandwidthHz > 0.0);
  NS_ASSERT(ctx.gapMinMs >= 0.0);
  NS_ASSERT(ctx.gapMaxMs >= 0.0);
  NS_ASSERT(ctx.gapMaxMs >= ctx.gapMinMs);
  NS_ASSERT(ctx.kTsym >= 0.0);
  NS_ASSERT(ctx.marginAlpha >= 0.0);

  const double tsymMs =
      1000.0 * static_cast<double>(1u << ctx.sf) / ctx.bandwidthHz;

  const bool sfGateActive =
      (ctx.sf >= ctx.sfAwarePacingThreshold);

  auto clampGap = [&](double g) -> double
  {
    return std::min(ctx.gapMaxMs, std::max(ctx.gapMinMs, g));
  };

  double severity = 0.0;
  double rawGapMs = 0.0;
  double gapMs = 0.0;

  switch (ctx.pacingMode)
    {
    case PacingMode::BASELINE:
      gapMs = 0.0;
      break;

    case PacingMode::FIXED:
      gapMs = ctx.fixedGapMs;
      break;

    case PacingMode::RADIOAWARE:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          severity = std::max(0.0, ctx.marginRefDb - ctx.marginDb);
          rawGapMs = ctx.kTsym * tsymMs *
                     (1.0 + ctx.marginAlpha * severity);
          gapMs = rawGapMs;
        }
      break;

    case PacingMode::RADIOAWARE_PHASE_BOUNDED:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          severity = std::max(0.0, ctx.marginRefDb - ctx.marginDb);
          rawGapMs = ctx.kTsym * tsymMs *
                     (1.0 + ctx.marginAlpha * severity);
          gapMs = clampGap(rawGapMs);
        }
      break;
     
    case PacingMode::STOCHASTIC:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          gapMs = m_uv->GetValue(0.0, static_cast<double>(m_gapMaxMs));
        }
      break;

    default:
      NS_FATAL_ERROR("Unknown pacing mode");
    }

  NS_ASSERT(gapMs >= 0.0);

  NS_LOG_INFO("COMPUTE_FRAGMENT_GAP"
              << " sf=" << static_cast<uint32_t>(ctx.sf)
              << " bandwidthHz=" << ctx.bandwidthHz
              << " pacingMode=" << static_cast<uint32_t>(ctx.pacingMode)
              << " sfAwarePacingThreshold="
              << static_cast<uint32_t>(ctx.sfAwarePacingThreshold)
              << " sfGateActive=" << sfGateActive
              << " marginDb=" << ctx.marginDb
              << " marginRefDb=" << ctx.marginRefDb
              << " marginAlpha=" << ctx.marginAlpha
              << " kTsym=" << ctx.kTsym
              << " tsymMs=" << tsymMs
              << " severity=" << severity
              << " rawGapMs=" << rawGapMs
              << " gapMinMs=" << ctx.gapMinMs
              << " gapMaxMs=" << ctx.gapMaxMs
              << " resultGapMs=" << gapMs);

  return gapMs;
}

double
PqcExchangeApplication::ComputeAckRecoveryGapMs(
    const AckRecoveryGapContext& ctx) const
{
  /*
   * Contract:
   *   Compute the pacing gap applied before ACK recovery redelivery rounds.
   *
   * Design goals:
   *   - preserve a radio-aware baseline tied to LoRa symbol time
   *   - allow ACK recovery to escape persistent bad RX-duty-cycle phases
   *   - break deterministic repetition of the same ACK failure phase
   *   - remain bounded and auditable under phase-bounded pacing
   *
   * High-level behavior:
   *
   *   BASELINE:
   *     no extra ACK-recovery gap
   *
   *   FIXED:
   *     constant configured recovery gap
   *
   *   RADIOAWARE:
   *     symbol-time-aware recovery gap scaled by recovery severity
   *
   *   RADIOAWARE_PHASE_BOUNDED:
   *     radio-aware recovery gap +
   *     RX-cycle-scale phase escape +
   *     uniform cycle jitter +
   *     bounded final gap
   *
   * Important:
   *   This method computes only the recovery-gap magnitude.
   *   Exact RX-window alignment is still enforced later at transmission time.
   */

  NS_ASSERT(ctx.bandwidthHz > 0.0);
  NS_ASSERT(ctx.gapMinMs >= 0.0);
  NS_ASSERT(ctx.gapMaxMs >= 0.0);
  NS_ASSERT(ctx.gapMaxMs >= ctx.gapMinMs);
  NS_ASSERT(ctx.kTsym >= 0.0);
  NS_ASSERT(ctx.ackRecoveryAlpha >= 0.0);
  NS_ASSERT(ctx.ackRecoveryBackoffGamma >= 0.0);
  NS_ASSERT(ctx.rxWindowMs >= 0.0);
  NS_ASSERT(ctx.rxIdleMs >= 0.0);

  if (ctx.recoveryRound == 0)
    {
      NS_LOG_INFO("COMPUTE_ACK_RECOVERY_GAP"
                  << " recoveryRound=0"
                  << " resultGapMs=0");
      return 0.0;
    }

  NS_ASSERT(ctx.recoveryRound > 0);

  const double tsymMs =
      1000.0 * static_cast<double>(1u << ctx.sf) / ctx.bandwidthHz;

  const bool sfGateActive =
      (ctx.sf >= ctx.sfAwarePacingThreshold);

  const double cycleMs = ctx.rxWindowMs + ctx.rxIdleMs;
  const bool hasDutyCycle =
      ctx.rxDutyCycleEnabled && (cycleMs > 0.0);

  auto clampGap = [&](double g) -> double
  {
    return std::min(ctx.gapMaxMs, std::max(ctx.gapMinMs, g));
  };

  const double severity =
      std::max(0.0,
               ctx.ackRecoveryMarginRefDb - ctx.ackRecoveryMarginDb);

  const double recoveryScale =
      1.0 + ctx.ackRecoveryBackoffGamma *
      static_cast<double>(ctx.recoveryRound - 1);

  const double radioAwareGapMs =
      ctx.kTsym * tsymMs *
      (1.0 + ctx.ackRecoveryAlpha * severity) *
      recoveryScale;

  double rawGapMs = 0.0;
  double phaseEscapeMs = 0.0;
  double jitterMs = 0.0;
  double gapMs = 0.0;
  double escapeFactor = 0.0;

  switch (ctx.pacingMode)
    {
    case PacingMode::BASELINE:
      gapMs = 0.0;
      break;

    case PacingMode::FIXED:
      gapMs = ctx.fixedGapMs;
      break;

    case PacingMode::RADIOAWARE:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          gapMs = radioAwareGapMs;
        }
      break;

    case PacingMode::RADIOAWARE_PHASE_BOUNDED:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          rawGapMs = radioAwareGapMs;

          if (hasDutyCycle)
            {
              NS_ABORT_MSG_IF(ctx.gapMaxMs < cycleMs,
                              "gapMaxMs must be >= RX cycle "
                              "(rxWindowMs + rxIdleMs) to enable "
                              "ACK-recovery phase escape under duty cycling.");

              escapeFactor =
                  std::min(1.0,
                           0.5 * static_cast<double>(ctx.recoveryRound));

              phaseEscapeMs = escapeFactor * cycleMs;

              jitterMs = m_uv->GetValue() * 0.5 * cycleMs;
            }

          gapMs = rawGapMs + phaseEscapeMs + jitterMs;
          gapMs = clampGap(gapMs);
        }
      break;
      
      
    case PacingMode::STOCHASTIC:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          /*
           * Randomized ACK-recovery gap:
           * use the same upper gap scale as bounded pacing, but sample
           * independently from a uniform distribution.
           */
          gapMs = m_uv->GetValue(0.0, ctx.gapMaxMs);
        }
      break;

    default:
      NS_FATAL_ERROR("Unknown pacing mode");
    }

  NS_ASSERT(gapMs >= 0.0);

  NS_LOG_INFO("COMPUTE_ACK_RECOVERY_GAP"
              << " sf=" << static_cast<uint32_t>(ctx.sf)
              << " bandwidthHz=" << ctx.bandwidthHz
              << " pacingMode=" << static_cast<uint32_t>(ctx.pacingMode)
              << " sfAwarePacingThreshold="
              << static_cast<uint32_t>(ctx.sfAwarePacingThreshold)
              << " sfGateActive=" << sfGateActive
              << " recoveryRound=" << ctx.recoveryRound
              << " ackRecoveryMarginDb=" << ctx.ackRecoveryMarginDb
              << " ackRecoveryMarginRefDb=" << ctx.ackRecoveryMarginRefDb
              << " ackRecoveryAlpha=" << ctx.ackRecoveryAlpha
              << " ackRecoveryBackoffGamma=" << ctx.ackRecoveryBackoffGamma
              << " severity=" << severity
              << " recoveryScale=" << recoveryScale
              << " tsymMs=" << tsymMs
              << " radioAwareGapMs=" << radioAwareGapMs
              << " rxDutyCycleEnabled=" << ctx.rxDutyCycleEnabled
              << " rxWindowMs=" << ctx.rxWindowMs
              << " rxIdleMs=" << ctx.rxIdleMs
              << " cycleMs=" << cycleMs
              << " escapeFactor=" << escapeFactor
              << " phaseEscapeMs=" << phaseEscapeMs
              << " jitterMs=" << jitterMs
              << " rawGapMs=" << rawGapMs
              << " gapMinMs=" << ctx.gapMinMs
              << " gapMaxMs=" << ctx.gapMaxMs
              << " resultGapMs=" << gapMs);

  return gapMs;
}

void
PqcExchangeApplication::RunOneFragment(ExchangeContext& ctx,
                                       uint32_t fragIndex)
{
  /*
   * Contract:
   *   Execute the full protocol logic for exactly one logical DATA fragment:
   *
   *     1. derive fragment payload sizes
   *     2. compute nominal PHY failure probability
   *     3. apply inter-fragment pacing if fragIndex > 0
   *     4. run ARQ for DATA delivery
   *     5. if DATA succeeds, enqueue fragment for ACK batching
   *     6. if ACK batch threshold is reached, attempt ACK flush
   *
   * Important:
   *   This method does NOT own terminal exchange-level success accounting.
   *
   * Current responsibilities are split as follows:
   *
   *   - RunFragmentArq():
   *       * counts terminal DATA failures
   *       * sets ctx.outcome.exchangeFailed on terminal DATA failure
   *
   *   - FlushAckBatch():
   *       * promotes DATA-delivered fragments to exchange-level success
   *         by incrementing ctx.outcome.okFrags when ACK succeeds
   *
   * Therefore this caller must never re-count fragment success/failure.
   */

  const uint32_t payloadCrypto = ctx.runtime.fragPayloads.at(fragIndex);
  const uint32_t payloadPhy = ComputePhyPayloadBytes(payloadCrypto);

  const double airtimeFragMs = ComputeFragmentAirtimeMs(payloadPhy);
  const double airtimeFragS = airtimeFragMs / 1000.0;

  /*
   * Nominal PHY/channel failure probability is computed once per fragment
   * payload and scenario SNR. Retries reuse the same nominal pPhy; only the
   * attempt time changes collision exposure and RX duty-cycle alignment.
   */
  const double pPhy =
      ComputeChannelFailureProbability(payloadPhy, ctx.out.snrDb);

  /*
   * Accumulate nominal per-fragment statistics independently of the eventual
   * stochastic terminal outcome.
   */
  AccumulateNominalFragmentStats(ctx,
                                 payloadPhy,
                                 airtimeFragMs,
                                 airtimeFragS,
                                 pPhy);

  double fragGapMs = 0.0;

  /*
   * Apply inter-fragment pacing between consecutive logical fragments.
   * The first fragment of an exchange has no preceding fragment gap.
   */
  if (fragIndex > 0)
    {
      FragmentGapContext fragCtx;
      fragCtx.sf = m_sf;
      fragCtx.bandwidthHz = static_cast<double>(m_bwHz);
      fragCtx.marginDb = ctx.out.marginDb;
      fragCtx.pacingMode = m_pacing;
      fragCtx.fixedGapMs = static_cast<double>(m_fixedGapMs);
      fragCtx.gapMinMs = static_cast<double>(m_gapMinMs);
      fragCtx.gapMaxMs = static_cast<double>(m_gapMaxMs);
      fragCtx.kTsym = m_kTsym;
      fragCtx.sfAwarePacingThreshold = m_sfAwarePacingThreshold;
      fragCtx.marginRefDb = m_marginRefDb;
      fragCtx.marginAlpha = m_marginAlpha;

      fragGapMs = ComputeFragmentGapMs(fragCtx);

      const double nowBeforeGapS = ctx.runtime.nowS;
      ctx.runtime.nowS += fragGapMs / 1000.0;
      ctx.stats.elapsedMsSum += fragGapMs;

      NS_LOG_INFO("APPLY_FRAGMENT_GAP"
                  << " fragIndex=" << fragIndex
                  << " gapMs=" << fragGapMs
                  << " nowBeforeS=" << nowBeforeGapS
                  << " nowAfterS=" << ctx.runtime.nowS);
    }

  /*
   * Exported snapshot of the last fragment-gap value applied by this exchange.
   * This preserves the current CSV contract.
   */
  ctx.out.fragGapMs = fragGapMs;

  PendingAckEntry e;
  e.fragIndex = fragIndex;
  e.airtimeFragMs = airtimeFragMs;
  e.pPhy = pPhy;
  e.retxUsed = 0;

  /*
   * Execute DATA ARQ.
   *
   * On success:
   *   - the fragment is delivered at DATA level
   *   - it is NOT yet an exchange-level success
   *
   * On failure:
   *   - RunFragmentArq() has already counted the terminal DATA failure
   *   - RunFragmentArq() has already set ctx.outcome.exchangeFailed = true
   *
   * Therefore this method must only stop control flow, without re-counting.
   */
  const bool dataDelivered = RunFragmentArq(ctx, e);

  if (!dataDelivered)
    {
      return;
    }

  /*
   * DATA delivery succeeded. The fragment now awaits logical acknowledgment
   * through ACK batching.
   */
  ctx.ack.pendingAckQueue.push_back(e);

  /*
   * Attempt ACK flush only when batching policy says so.
   *
   * If FlushAckBatch() returns false, ACK handling failed terminally and the
   * exchange has already been marked as failed by the ACK subsystem.
   */
  const bool flushOk = FlushAckBatch(ctx, false);

  if (!flushOk)
    {
      ctx.outcome.exchangeFailed = true;
      return;
    }
}

double
PqcExchangeApplication::ComputeDataAttemptGapMs(
    const DataAttemptGapContext& ctx) const
{
  /*
   * Contract:
   *   Compute the pacing gap applied before a DATA retransmission attempt.
   *
   * Design goals:
   *   - preserve a radio-aware baseline tied to LoRa symbol time
   *   - allow retries to escape bad RX-duty-cycle phase regions
   *   - break deterministic repetition of the same phase
   *   - provide a strong and causally clear mechanism under duty cycling
   *
   * High-level behavior:
   *
   *   BASELINE:
   *     no extra retry gap
   *
   *   FIXED:
   *     constant configured retry gap
   *
   *   RADIOAWARE:
   *     radio-aware retry gap based on Tsym and retry severity
   *
   *   RADIOAWARE_PHASE_BOUNDED:
   *     radio-aware retry gap +
   *     phase-escape jump on RX-cycle scale +
   *     random jitter over half the RX cycle +
   *     bounded final gap
   *
   * Important:
   *   This method computes only the retry gap magnitude.
   *   Exact RX-window alignment is still enforced later at attempt time by
   *   AlignToRxWindowStartForFullContainment(...).
   */

  NS_ASSERT(ctx.bandwidthHz > 0.0);
  NS_ASSERT(ctx.gapMinMs >= 0.0);
  NS_ASSERT(ctx.gapMaxMs >= 0.0);
  NS_ASSERT(ctx.gapMaxMs >= ctx.gapMinMs);
  NS_ASSERT(ctx.kTsym >= 0.0);
  NS_ASSERT(ctx.retryAlpha >= 0.0);
  NS_ASSERT(ctx.retryBackoffGamma >= 0.0);
  NS_ASSERT(ctx.rxWindowMs >= 0.0);
  NS_ASSERT(ctx.rxIdleMs >= 0.0);
  NS_ASSERT(ctx.attemptIndex > 0);

  const double tsymMs =
      1000.0 * static_cast<double>(1u << ctx.sf) / ctx.bandwidthHz;

  const bool sfGateActive =
      (ctx.sf >= ctx.sfAwarePacingThreshold);

  const double cycleMs = ctx.rxWindowMs + ctx.rxIdleMs;
  const bool hasDutyCycle =
      ctx.rxDutyCycleEnabled && (cycleMs > 0.0);

  auto clampGap = [&](double g) -> double
  {
    return std::min(ctx.gapMaxMs, std::max(ctx.gapMinMs, g));
  };

  /*
   * Retry severity:
   *   larger when retry margin is poor.
   */
  const double severity =
      std::max(0.0, ctx.retryMarginRefDb - ctx.retryMarginDb);

  /*
   * Retry backoff scale:
   *   grows with retry index so later retries move more aggressively.
   *
   * attemptIndex:
   *   1 = first retry
   *   2 = second retry
   *   ...
   */
  const double retryScale =
      1.0 + ctx.retryBackoffGamma * static_cast<double>(ctx.attemptIndex);

  /*
   * Base radio-aware component tied to LoRa symbol time.
   */
  const double radioAwareGapMs =
      ctx.kTsym * tsymMs *
      (1.0 + ctx.retryAlpha * severity) *
      retryScale;

  double rawGapMs = 0.0;
  double phaseEscapeMs = 0.0;
  double jitterMs = 0.0;
  double gapMs = 0.0;
  double escapeFactor = 0.0;

  switch (ctx.pacingMode)
    {
    case PacingMode::BASELINE:
      gapMs = 0.0;
      break;

    case PacingMode::FIXED:
      gapMs = ctx.fixedGapMs;
      break;

    case PacingMode::RADIOAWARE:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          /*
           * Radio-aware only:
           * improves spacing, but does not explicitly try to escape RX phase.
           */
          gapMs = radioAwareGapMs;
        }
      break;

    case PacingMode::RADIOAWARE_PHASE_BOUNDED:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          rawGapMs = radioAwareGapMs;

          if (hasDutyCycle)
            {
              NS_ABORT_MSG_IF(ctx.gapMaxMs < cycleMs,
                              "gapMaxMs must be >= RX cycle (rxWindowMs + rxIdleMs) "
                              "to allow phase escape under duty cycling.");

              escapeFactor =
                  std::min(1.0,
                           0.5 * static_cast<double>(ctx.attemptIndex));

              phaseEscapeMs = escapeFactor * cycleMs;

              /*
               * Controlled jitter over half a cycle:
               * enough to break exact phase repetition without excessive variance.
               */
              jitterMs = m_uv->GetValue() * 0.5 * cycleMs;
            }

          gapMs = rawGapMs + phaseEscapeMs + jitterMs;
          gapMs = clampGap(gapMs);
        }
      break;
    case PacingMode::STOCHASTIC:
      if (!sfGateActive)
        {
          gapMs = 0.0;
        }
      else
        {
          /*
           * Randomized retry baseline:
           * draw one independent inter-attempt gap uniformly over
           * [0, gapMaxMs]. This preserves the same upper gap scale
           * used by bounded pacing, but removes deterministic phase evolution.
           */
          gapMs = m_uv->GetValue(0.0, ctx.gapMaxMs);
        }
      break;
      
    default:
      NS_FATAL_ERROR("Unknown pacing mode");
    }

  NS_ASSERT(gapMs >= 0.0);

  NS_LOG_INFO("COMPUTE_DATA_ATTEMPT_GAP"
              << " sf=" << static_cast<uint32_t>(ctx.sf)
              << " bandwidthHz=" << ctx.bandwidthHz
              << " pacingMode=" << static_cast<uint32_t>(ctx.pacingMode)
              << " sfAwarePacingThreshold="
              << static_cast<uint32_t>(ctx.sfAwarePacingThreshold)
              << " sfGateActive=" << sfGateActive
              << " attemptIndex=" << ctx.attemptIndex
              << " retryMarginDb=" << ctx.retryMarginDb
              << " retryMarginRefDb=" << ctx.retryMarginRefDb
              << " retryAlpha=" << ctx.retryAlpha
              << " retryBackoffGamma=" << ctx.retryBackoffGamma
              << " severity=" << severity
              << " retryScale=" << retryScale
              << " tsymMs=" << tsymMs
              << " radioAwareGapMs=" << radioAwareGapMs
              << " rxDutyCycleEnabled=" << ctx.rxDutyCycleEnabled
              << " rxWindowMs=" << ctx.rxWindowMs
              << " rxIdleMs=" << ctx.rxIdleMs
              << " cycleMs=" << cycleMs
              << " escapeFactor=" << escapeFactor
              << " phaseEscapeMs=" << phaseEscapeMs
              << " jitterMs=" << jitterMs
              << " rawGapMs=" << rawGapMs
              << " gapMinMs=" << ctx.gapMinMs
              << " gapMaxMs=" << ctx.gapMaxMs
              << " resultGapMs=" << gapMs);

  return gapMs;
}

void
PqcExchangeApplication::ValidateConfiguration() const
{
  /*
   * Contract:
   *   Validate the public configuration of the exchange application before
   *   running any exchange.
   *
   * Philosophy:
   *   - reject invalid or ambiguous protocol states early
   *   - enforce only scientifically meaningful constraints
   *   - keep internal tuning constants out of the public contract
   */

  // --------------------------------------------------------------------------
  // Core identity / PHY
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(m_sf < 7 || m_sf > 12,
                  "Invalid Sf: supported LoRa SF range is [7, 12].");

  NS_ABORT_MSG_IF(m_bwHz == 0,
                  "Invalid BwHz: bandwidth must be > 0 Hz.");

  NS_ABORT_MSG_IF(m_crDen < 5 || m_crDen > 8,
                  "Invalid CrDen: supported coding-rate denominator range is [5, 8].");

  // --------------------------------------------------------------------------
  // Fragmentation / payload contract
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(m_payloadBytes == 0,
                  "Invalid PayloadBytes: payload must be > 0 bytes.");

  NS_ABORT_MSG_IF(m_g2gOverheadBytes == 0,
                  "Invalid G2gOverheadBytes: overhead must be > 0 bytes.");

  if (!m_autoFragments)
    {
      NS_ABORT_MSG_IF((m_pkFragments + m_ctFragments) == 0,
                      "Invalid manual fragmentation: PkFragments + CtFragments must be > 0.");
    }
  else
    {
      NS_ABORT_MSG_IF(m_pkBytes == 0 && m_ctBytes == 0,
                      "Invalid auto fragmentation: at least one of PkBytes or CtBytes must be > 0.");

      NS_ABORT_MSG_IF(m_maxPayloadOverrideBytes > 0 &&
                          m_maxPayloadOverrideBytes <= m_g2gOverheadBytes,
                      "Invalid MaxPayloadOverrideBytes: PHY payload cap must exceed G2gOverheadBytes so that at least 1 crypto byte can fit.");
    }

  // --------------------------------------------------------------------------
  // Pacing
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(m_kTsym < 0.0,
                  "Invalid KTsym: must be >= 0.");

  NS_ABORT_MSG_IF(m_gapMinMs > m_gapMaxMs,
                  "Invalid bounded pacing configuration: GapMinMs must be <= GapMaxMs.");

  NS_ABORT_MSG_IF(!(m_sfAwarePacingThreshold == 255 ||
                    (m_sfAwarePacingThreshold >= 7 && m_sfAwarePacingThreshold <= 12)),
                  "Invalid SfAwarePacingThreshold: must be 255 or in the SF range [7, 12].");

  if (m_pacing == PacingMode::FIXED)
    {
      // fixedGapMs is allowed to be zero; that simply means no added gap
    }

  if (m_pacing == PacingMode::RADIOAWARE_PHASE_BOUNDED && m_rxDutyCycleEnabled)
    {
      const double cycleMs = m_rxWindowMs + m_rxIdleMs;

      NS_ABORT_MSG_IF(cycleMs <= 0.0,
                      "Invalid RX duty-cycle configuration for phase-bounded pacing: rxWindowMs + rxIdleMs must be > 0.");

      NS_ABORT_MSG_IF(m_gapMaxMs < cycleMs,
                      "Invalid GapMaxMs for radioaware_phase_bounded under RX duty cycling: GapMaxMs must be >= rxWindowMs + rxIdleMs so retries can escape bad RX phases.");
    }

  // --------------------------------------------------------------------------
  // Retry / ACK / RX duty cycle
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(m_ackBatchSize == 0,
                  "Invalid AckBatchSize: must be >= 1.");

  NS_ABORT_MSG_IF(m_rxWindowMs < 0.0 || m_rxIdleMs < 0.0,
                  "Invalid RX duty-cycle parameters: RxWindowMs and RxIdleMs must be >= 0.");

  if (m_rxDutyCycleEnabled)
    {
      NS_ABORT_MSG_IF(m_rxWindowMs <= 0.0,
                      "Invalid RX duty-cycle configuration: RxWindowMs must be > 0 when RxDutyCycleEnabled=true.");
    }

  // --------------------------------------------------------------------------
  // Scenario SNR / link budget
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(m_noiseFigureDb < 0.0,
                  "Invalid NoiseFigureDb: must be >= 0 dB.");

  NS_ABORT_MSG_IF(m_distanceM < 0.0,
                  "Invalid DistanceM: must be >= 0 m.");

  NS_ABORT_MSG_IF(m_refDistanceM <= 0.0,
                  "Invalid RefDistanceM: must be > 0 m.");

  NS_ABORT_MSG_IF(m_refPathLossDb < 0.0,
                  "Invalid RefPathLossDb: must be >= 0 dB.");

  NS_ABORT_MSG_IF(m_pathLossExp <= 0.0,
                  "Invalid PathLossExp: must be > 0.");

  NS_ABORT_MSG_IF(m_shadowingSigmaDb < 0.0,
                  "Invalid ShadowingSigmaDb: must be >= 0 dB.");

  // --------------------------------------------------------------------------
  // Energy model
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(m_txEnergyJPerMs < 0.0,
                  "Invalid TxEnergyJPerMs: must be >= 0.");

  NS_ABORT_MSG_IF(m_rxEnergyJPerMs < 0.0,
                  "Invalid RxEnergyJPerMs: must be >= 0.");

  // --------------------------------------------------------------------------
  // Scenario label consistency
  // --------------------------------------------------------------------------
  NS_ABORT_MSG_IF(!(m_environment == "indoor" || m_environment == "outdoor"),
                  "Invalid Environment: supported values are 'indoor' and 'outdoor'.");
}

void
PqcExchangeApplication::ApplyRetryGap(ExchangeContext& ctx,
                                      PendingAckEntry& e,
                                      double attemptEndS,
                                      const std::string& cause)
{
  /*
   * Contract:
   *   Apply retry pacing after one non-terminal DATA attempt failure.
   *
   * Inputs:
   *   ctx         : exchange runtime/state container
   *   e           : fragment state for the fragment being retried
   *   attemptEndS : end time (in seconds) of the failed DATA attempt
   *   cause       : diagnostic label describing why the retry is being applied
   *                 (e.g., "rx_sleep", "channel_or_collision")
   *
   * Responsibilities:
   *   - build the retry-gap context from the current application configuration
   *   - compute the retry gap using the active pacing policy
   *   - advance exchange runtime time to:
   *
   *         attemptEndS + retryGap
   *
   *   - accumulate elapsed-time accounting for the retry gap
   *   - emit a reproducible diagnostic log entry
   *
   * Important:
   *   - This function does NOT change retry counters.
   *     Retry counters must already have been updated by the caller.
   *
   *   - This function does NOT modify fragment success/failure accounting.
   *     It only advances simulated time between attempts.
   *
   *   - The retry gap is part of exchange latency but not TX airtime.
   *     Therefore:
   *       * ctx.stats.elapsedMsSum is incremented
   *       * ctx.stats.txAirtimeMsSum is NOT incremented
   *
   *   - attemptIndex is derived from e.retxUsed, which is expected to already
   *     reflect the retry number being scheduled.
   */

  DataAttemptGapContext gapCtx;
  gapCtx.sf = m_sf;
  gapCtx.bandwidthHz = static_cast<double>(m_bwHz);
  gapCtx.retryMarginDb = m_retryMarginDb;
  gapCtx.attemptIndex = e.retxUsed;
  gapCtx.pacingMode = m_pacing;
  gapCtx.fixedGapMs = static_cast<double>(m_fixedGapMs);
  gapCtx.gapMinMs = static_cast<double>(m_gapMinMs);
  gapCtx.gapMaxMs = static_cast<double>(m_gapMaxMs);
  gapCtx.kTsym = m_kTsym;
  gapCtx.sfAwarePacingThreshold = m_sfAwarePacingThreshold;
  gapCtx.retryMarginRefDb = m_retryMarginRefDb;
  gapCtx.retryAlpha = m_retryAlpha;
  gapCtx.retryBackoffGamma = m_retryBackoffGamma;
  gapCtx.rxDutyCycleEnabled = m_rxDutyCycleEnabled;
  gapCtx.rxWindowMs = m_rxWindowMs;
  gapCtx.rxIdleMs = m_rxIdleMs;

  const double attemptGapMs = ComputeDataAttemptGapMs(gapCtx);

  const double nowBeforeS = ctx.runtime.nowS;
  ctx.runtime.nowS = attemptEndS + attemptGapMs / 1000.0;
  ctx.stats.elapsedMsSum += attemptGapMs;

  NS_LOG_INFO("APPLY_DATA_ATTEMPT_GAP"
              << " fragIndex=" << e.fragIndex
              << " retxUsed=" << e.retxUsed
              << " cause=" << cause
              << " gapMs=" << attemptGapMs
              << " nowBeforeS=" << nowBeforeS
              << " attemptEndS=" << attemptEndS
              << " nowAfterS=" << ctx.runtime.nowS);
}


bool
PqcExchangeApplication::RunFragmentArq(ExchangeContext& ctx,
                                       PendingAckEntry& e)
{
  /*
   * Contract:
   *   Execute stop-and-wait ARQ for exactly one DATA fragment until:
   *
   *     - the DATA fragment is delivered successfully, or
   *     - retransmission budget is exhausted.
   *
   * Responsibilities:
   *   - count real on-air DATA attempts
   *   - evaluate deterministic RX duty-cycle gating first
   *   - evaluate stochastic PHY/collision failure second
   *   - apply retry pacing between failed attempts
   *   - accumulate attempt-level airtime and retry diagnostics
   *   - accumulate fragment-level collision/total probability diagnostics
   *     EXACTLY ONCE, using the terminal attempt of the fragment:
   *       * successful terminal attempt, or
   *       * terminal failed attempt after retry exhaustion
   *   - update terminal DATA failure accounting exactly once:
   *       * terminal failure -> ctx.outcome.failFrags++,
   *                             ctx.outcome.exchangeFailed = true
   *
   * Important:
   *   DATA success here is NOT yet an exchange-level success.
   *   Exchange-level success is granted only when the corresponding ACK batch
   *   is successfully received in FlushAckBatch().
   *
   * Semantics of collision/total averages:
   *   - ctx.stats.sumColSim, sumColTh, sumTot are FRAGMENT-level aggregates
   *   - therefore they MUST NOT be accumulated on every retry attempt
   *   - otherwise, dividing them by nFrags in FinalizeExchange() can yield
   *     averages > 1.0, which is semantically invalid for probabilities
   *
   * Return value:
   *   true  -> DATA delivery succeeded
   *   false -> DATA fragment failed terminally after exhausting retries
   */

  NS_ASSERT(e.airtimeFragMs >= 0.0);
  NS_ASSERT(std::isfinite(e.airtimeFragMs));
  NS_ASSERT(e.pPhy >= 0.0 && e.pPhy <= 1.0);
  NS_ASSERT(std::isfinite(e.pPhy));
  NS_ASSERT(ctx.runtime.logicalChannels > 0);

  while (true)
    {
      double attemptNowS = ctx.runtime.nowS;

      NS_ASSERT(std::isfinite(attemptNowS));
      NS_ASSERT(attemptNowS >= 0.0);

      /*
       * In phase-bounded mode under RX duty cycling, align the DATA attempt to
       * the next RX-active instant that can fully contain the DATA frame.
       */
      if (m_rxDutyCycleEnabled &&
                    (m_pacing == PacingMode::RADIOAWARE_PHASE_BOUNDED ||
                     m_pacing == PacingMode::STOCHASTIC))
        {
          attemptNowS = AlignToRxWindowStartForFullContainment(attemptNowS,
                                                               e.airtimeFragMs,
                                                               m_rxWindowMs,
                                                               m_rxIdleMs);
          ctx.runtime.nowS = attemptNowS;
        }

      const double attemptEndS =
          attemptNowS + e.airtimeFragMs / 1000.0;

      NS_ASSERT(std::isfinite(attemptEndS));
      NS_ASSERT(attemptEndS >= attemptNowS);

      /*
       * Count one real on-air DATA transmission attempt.
       * This is the authoritative counter for CSV field data_frames_tx.
       */
      ++ctx.outcome.dataFramesTxReal;

      /*
       * Accumulate burst/load diagnostics at the actual attempt time.
       */
      AccumulateAttemptDiagnostics(ctx, attemptNowS);

      /*
       * Compute collision-related failure probability for this attempt.
       * The nominal PHY failure probability e.pPhy was precomputed for the
       * fragment payload and scenario SNR.
       */
      const double pCol =
          ComputeCollisionFailureProbability(attemptNowS,
                                             e.airtimeFragMs,
                                             ctx.runtime.logicalChannels);

      NS_ASSERT(std::isfinite(pCol));
      NS_ASSERT(pCol >= 0.0 && pCol <= 1.0);

      /*
       * Contractual total failure probability:
       *
       *   p_total = 1 - (1 - p_phy) * (1 - p_collision)
       */
      const double pTot =
          ComputeTotalFailureProbability(e.pPhy, pCol);

      NS_ASSERT(std::isfinite(pTot));
      NS_ASSERT(pTot >= 0.0 && pTot <= 1.0);

      /*
       * Attempt-level theoretical collision approximation evaluated at the
       * same realized attempt timestamp and airtime.
       *
       * IMPORTANT:
       *   This is used as a fragment-level comparable diagnostic, so it must
       *   be accumulated only on the terminal attempt of the fragment.
       */
      double pColTh = 0.0;

      if (m_col)
        {
          const uint32_t logicalChannelsEff = ctx.runtime.logicalChannels;
          NS_ASSERT(logicalChannelsEff > 0);

          const double lambdaEff =
              m_col->GetEffectiveLambdaTotal(attemptNowS);

          NS_ASSERT(std::isfinite(lambdaEff));
          NS_ASSERT(lambdaEff >= 0.0);

          const double lambdaPerChannel =
              lambdaEff / static_cast<double>(logicalChannelsEff);
          const double airtimeFragS = e.airtimeFragMs / 1000.0;

          NS_ASSERT(std::isfinite(lambdaPerChannel));
          NS_ASSERT(lambdaPerChannel >= 0.0);
          NS_ASSERT(std::isfinite(airtimeFragS));
          NS_ASSERT(airtimeFragS >= 0.0);

          pColTh = 1.0 - std::exp(-2.0 * lambdaPerChannel * airtimeFragS);
          pColTh = std::clamp(pColTh, 0.0, 1.0);

          NS_ASSERT(std::isfinite(pColTh));
          NS_ASSERT(pColTh >= 0.0 && pColTh <= 1.0);
        }

      /*
       * Every real DATA attempt contributes full DATA airtime.
       *
       * This is intentionally attempt-level, because airtime and latency are
       * real resource consumptions for every retransmission.
       */
      ctx.stats.txAirtimeMsSum += e.airtimeFragMs;
      ctx.stats.elapsedMsSum += e.airtimeFragMs;

      NS_ASSERT(std::isfinite(ctx.stats.txAirtimeMsSum));
      NS_ASSERT(std::isfinite(ctx.stats.elapsedMsSum));

      /*
       * Deterministic RX duty-cycle gating with full-window containment.
       *
       * The full frame interval must fit entirely inside one active RX window.
       * Partial overlap does not count as successful reception.
       */
      bool rxWindowOpen = true;
      if (m_rxDutyCycleEnabled)
        {
          rxWindowOpen =
              IsFrameFullyContainedInRxWindow(attemptNowS * 1000.0,
                                              e.airtimeFragMs,
                                              m_rxWindowMs,
                                              m_rxIdleMs);
        }

      NS_LOG_INFO("ATTEMPT_BEGIN"
                  << " fragIndex=" << e.fragIndex
                  << " retxUsed=" << e.retxUsed
                  << " attemptNowS=" << attemptNowS
                  << " attemptEndS=" << attemptEndS
                  << " airtimeFragMs=" << e.airtimeFragMs
                  << " pPhy=" << e.pPhy
                  << " pCol=" << pCol
                  << " pColTh=" << pColTh
                  << " pTot=" << pTot
                  << " rxDutyCycleEnabled=" << m_rxDutyCycleEnabled
                  << " rxWindowOpen=" << rxWindowOpen
                  << " dataFramesTxReal=" << ctx.outcome.dataFramesTxReal);

      /*
       * Deterministic RX-sleep failure path.
       */
      if (m_rxDutyCycleEnabled && !rxWindowOpen)
        {
          ctx.stats.lostRxSleepData++;

          NS_LOG_INFO("ATTEMPT_RESULT"
                      << " fragIndex=" << e.fragIndex
                      << " retxUsed=" << e.retxUsed
                      << " result=fail"
                      << " cause=rx_sleep"
                      << " lostRxSleepData=" << ctx.stats.lostRxSleepData);

          /*
           * Terminal DATA failure after exhausting retry budget.
           *
           * IMPORTANT:
           *   Collision/total probability diagnostics are accumulated HERE
           *   exactly once for the fragment, because this is the terminal
           *   attempt that defines the fragment's final DATA outcome.
           */
          if (e.retxUsed >= m_maxRetries)
            {
              ctx.stats.sumColSim += pCol;
              ctx.stats.sumColTh += pColTh;
              ctx.stats.sumTot += pTot;

              NS_ASSERT(std::isfinite(ctx.stats.sumColSim));
              NS_ASSERT(std::isfinite(ctx.stats.sumColTh));
              NS_ASSERT(std::isfinite(ctx.stats.sumTot));

              ctx.outcome.failFrags++;
              ctx.outcome.exchangeFailed = true;

              NS_LOG_INFO("ATTEMPT_RESULT"
                          << " fragIndex=" << e.fragIndex
                          << " retxUsed=" << e.retxUsed
                          << " result=terminal_fail"
                          << " cause=rx_sleep"
                          << " failFrags=" << ctx.outcome.failFrags
                          << " exchangeFailed=" << ctx.outcome.exchangeFailed
                          << " terminalAccumulatedPCol=" << pCol
                          << " terminalAccumulatedPColTh=" << pColTh
                          << " terminalAccumulatedPTot=" << pTot);

              return false;
            }

          /*
           * Non-terminal failure: consume one retry and apply retry pacing.
           *
           * IMPORTANT:
           *   Do NOT accumulate sumColSim / sumColTh / sumTot here.
           *   These are fragment-level averages exported per logical fragment,
           *   not per-attempt exposure counters.
           */
          ++e.retxUsed;
          ++ctx.outcome.totalRetx;

          ApplyRetryGap(ctx, e, attemptEndS, "rx_sleep");
          continue;
        }

      /*
       * Stochastic PHY/collision failure path.
       */
      const bool failed = (m_uv->GetValue() < pTot);
      if (failed)
        {
          NS_LOG_INFO("ATTEMPT_RESULT"
                      << " fragIndex=" << e.fragIndex
                      << " retxUsed=" << e.retxUsed
                      << " result=fail"
                      << " cause=channel_or_collision"
                      << " pPhy=" << e.pPhy
                      << " pCol=" << pCol
                      << " pColTh=" << pColTh
                      << " pTot=" << pTot);

          /*
           * Terminal DATA failure after exhausting retry budget.
           *
           * IMPORTANT:
           *   Accumulate fragment-level probability diagnostics exactly once
           *   on the terminal attempt.
           */
          if (e.retxUsed >= m_maxRetries)
            {
              ctx.stats.sumColSim += pCol;
              ctx.stats.sumColTh += pColTh;
              ctx.stats.sumTot += pTot;

              NS_ASSERT(std::isfinite(ctx.stats.sumColSim));
              NS_ASSERT(std::isfinite(ctx.stats.sumColTh));
              NS_ASSERT(std::isfinite(ctx.stats.sumTot));

              ctx.outcome.failFrags++;
              ctx.outcome.exchangeFailed = true;

              NS_LOG_INFO("ATTEMPT_RESULT"
                          << " fragIndex=" << e.fragIndex
                          << " retxUsed=" << e.retxUsed
                          << " result=terminal_fail"
                          << " cause=channel_or_collision"
                          << " failFrags=" << ctx.outcome.failFrags
                          << " exchangeFailed=" << ctx.outcome.exchangeFailed
                          << " pPhy=" << e.pPhy
                          << " pCol=" << pCol
                          << " pColTh=" << pColTh
                          << " pTot=" << pTot);

              return false;
            }

          /*
           * Non-terminal failure: consume one retry and apply retry pacing.
           *
           * IMPORTANT:
           *   Do NOT accumulate fragment-level collision/total diagnostics yet.
           *   Retry attempts are real airtime events, but they do not define
           *   the exported per-fragment nominal averages.
           */
          ++e.retxUsed;
          ++ctx.outcome.totalRetx;

          ApplyRetryGap(ctx, e, attemptEndS, "channel_or_collision");
          continue;
        }

      /*
       * DATA delivery succeeded.
       *
       * IMPORTANT:
       *   Accumulate fragment-level collision/theoretical/total probabilities
       *   exactly once here, because this successful attempt is the terminal
       *   DATA attempt for this logical fragment.
       *
       * Do NOT increment ctx.outcome.okFrags here.
       * Exchange-level success is granted only after successful ACK handling
       * in FlushAckBatch().
       */
      ctx.stats.sumColSim += pCol;
      ctx.stats.sumColTh += pColTh;
      ctx.stats.sumTot += pTot;

      NS_ASSERT(std::isfinite(ctx.stats.sumColSim));
      NS_ASSERT(std::isfinite(ctx.stats.sumColTh));
      NS_ASSERT(std::isfinite(ctx.stats.sumTot));

      ctx.runtime.nowS = attemptEndS;

      NS_LOG_INFO("ATTEMPT_RESULT"
                  << " fragIndex=" << e.fragIndex
                  << " retxUsed=" << e.retxUsed
                  << " result=data_success"
                  << " nowAfterS=" << ctx.runtime.nowS
                  << " pPhy=" << e.pPhy
                  << " pCol=" << pCol
                  << " pColTh=" << pColTh
                  << " pTot=" << pTot
                  << " terminalAccumulatedPCol=" << pCol
                  << " terminalAccumulatedPColTh=" << pColTh
                  << " terminalAccumulatedPTot=" << pTot);

      return true;
    }
}

PqcExchangeApplication::ExchangeContext
PqcExchangeApplication::InitializeExchange(double nowS,
                                           uint32_t logicalChannels) const
{
  /*
   * Contract:
   *   Build and return a fresh per-exchange execution context.
   *
   * Responsibilities:
   *   - initialize the exchange time origin
   *   - store the logical channel count used by this exchange
   *   - build the fragment plan for the full PQC exchange
   *   - initialize exported metadata fields
   *   - initialize scenario/link diagnostics
   *
   * Important:
   *   This method only prepares the exchange context.
   *   It does NOT:
   *     - execute fragment transmissions,
   *     - apply ARQ,
   *     - apply ACK logic,
   *     - finalize outcomes.
   */

  NS_ASSERT(logicalChannels > 0);

  ExchangeContext ctx;

  /*
   * Base time origin of this logical exchange.
   *
   * exchangeStartS:
   *   absolute time at which the exchange is instantiated.
   *
   * nowS:
   *   mutable protocol clock that will advance as fragment gaps, airtime,
   *   retransmissions, ACKs, and recovery rounds are executed.
   */
  ctx.runtime.exchangeStartS = nowS;
  ctx.runtime.nowS = nowS;

  /*
   * Number of logical channels exposed to the collision model for this
   * exchange execution.
   */
  ctx.runtime.logicalChannels = logicalChannels;

  /*
   * Build the full fragment plan for the exchange:
   *   PK fragments + CT fragments
   */
  BuildFragmentPlan(ctx.runtime.fragPayloads);
  ctx.runtime.nFrags = static_cast<uint32_t>(ctx.runtime.fragPayloads.size());

  /*
   * Initialize exported metadata snapshot and scenario diagnostics before the
   * protocol starts mutating runtime counters.
   */
  InitializeExchangeMetadata(ctx);
  InitializeScenarioDiagnostics(ctx);

  NS_LOG_INFO("INITIALIZE_EXCHANGE_CONTEXT"
              << " exchangeStartS=" << ctx.runtime.exchangeStartS
              << " nowS=" << ctx.runtime.nowS
              << " logicalChannels=" << ctx.runtime.logicalChannels
              << " nFrags=" << ctx.runtime.nFrags
              << " pkFragments=" << ctx.out.pkFragments
              << " ctFragments=" << ctx.out.ctFragments
              << " payloadBytesConfig=" << ctx.out.payloadBytesConfig
              << " payloadBytesEffective=" << ctx.out.payloadBytesEffective
              << " snrDb=" << ctx.out.snrDb
              << " rxDbm=" << ctx.out.rxDbm
              << " marginDb=" << ctx.out.marginDb
              << " lambdaEff=" << ctx.out.lambdaEff
              << " isOn=" << ctx.out.isOn);

  return ctx;
}

PqcExchangeApplication::ExchangeResult
PqcExchangeApplication::RunOneExchange(double nowS, uint32_t logicalChannels)
{
  /*
   * Contract:
   *   Execute one full logical PQC exchange and return exactly one
   *   ExchangeResult.
   *
   * Exchange-level semantics:
   *   - okFrags counts fragments that are successful at exchange level
   *   - failFrags counts all other logical fragments
   *   - therefore, at finalization:
   *
   *       okFrags + failFrags == nFrags
   *
   * Important:
   *   DATA-level success and exchange-level success are not identical.
   *   A fragment may succeed at DATA level, remain pending logical ACK, and
   *   later be classified as failed if the exchange aborts before that ACK
   *   succeeds.
   */

  ValidateConfiguration();

  ExchangeContext ctx = InitializeExchange(nowS, logicalChannels);

  /*
   * Empty exchanges are treated as structurally invalid and therefore marked
   * as failed at exchange level.
   *
   * Note:
   *   This is an exchange-level invalidity, not a fragment-loss event.
   *   Therefore failFrags remains 0 because no logical fragments existed.
   */
  if (ctx.runtime.nFrags == 0)
    {
      ctx.out.perExchangeFail = true;
      ctx.out.okFrags = 0;
      ctx.out.failFrags = 0;
      ctx.out.perObserved = 0.0;
      ctx.out.phyPayloadBytesFragAvg = 0.0;
      return ctx.out;
    }

  uint32_t nextFragIndex = 0;

  /*
   * Execute logical fragments until:
   *   - all fragments complete, or
   *   - a terminal exchange failure occurs.
   */
  for (; nextFragIndex < ctx.runtime.nFrags; ++nextFragIndex)
    {
      RunOneFragment(ctx, nextFragIndex);

      NS_LOG_INFO("RUN_FRAGMENT index=" << nextFragIndex
                  << " exchangeFailed=" << ctx.outcome.exchangeFailed);

      if (ctx.outcome.exchangeFailed)
        {
          NS_LOG_INFO("EXCHANGE_ABORT after fragIndex=" << nextFragIndex);
          break;
        }
    }

  /*
   * If all DATA fragments completed without terminal failure, force a final
   * ACK flush for any remaining batched acknowledgments.
   */
  if (!ctx.outcome.exchangeFailed && !FlushAckBatch(ctx, true))
    {
      ctx.outcome.exchangeFailed = true;
    }

  /*
   * If the exchange failed, reconcile logical fragment outcomes at exchange
   * level.
   *
   * Two classes of fragments must be counted as failed:
   *
   *   1. Fragments already delivered at DATA level but still pending logical
   *      ACK when the exchange aborted.
   *
   *   2. Fragments never started because the exchange aborted early.
   *
   * Pending-ACK fragments are not yet exchange-level successes, so they are
   * added directly to failFrags.
   */
  if (ctx.outcome.exchangeFailed)
    {
      const uint32_t pendingCount =
          static_cast<uint32_t>(ctx.ack.pendingAckQueue.size());
      ctx.ack.pendingAckQueue.clear();

      const uint32_t remainingNotStarted =
          (nextFragIndex + 1 < ctx.runtime.nFrags)
              ? (ctx.runtime.nFrags - (nextFragIndex + 1))
              : 0;

      ctx.outcome.failFrags += pendingCount + remainingNotStarted;
    }

  NS_ASSERT(ctx.ack.pendingAckQueue.empty());
  NS_ASSERT(ctx.outcome.okFrags + ctx.outcome.failFrags == ctx.runtime.nFrags);

  FinalizeExchange(ctx);
  return ctx.out;
}


void
PqcExchangeApplication::SetVariant(std::string v)
{
  /*
   * Contract:
   *   Set the experiment variant label exported in ExchangeResult / CSV.
   *
   * Behavior:
   *   - accepts any non-empty or empty string
   *   - does not alter simulator semantics directly
   *
   * Important:
   *   - Variant is treated as experiment metadata, not as a protocol control.
   *   - No validation is enforced here because variant naming is left to the
   *     campaign driver / user.
   */

  m_variant = std::move(v);
}

std::string
PqcExchangeApplication::GetVariant() const
{
  /*
   * Contract:
   *   Return the current experiment variant label.
   */

  return m_variant;
}

void
PqcExchangeApplication::SetEnvironment(std::string v)
{
  /*
   * Contract:
   *   Set the scenario environment label used by the application.
   *
   * Supported values:
   *     "indoor"
   *     "outdoor"
   *
   * Behavior:
   *   - accepts only the supported canonical environment strings
   *   - aborts on invalid values
   *
   * Important:
   *   - Environment is not free metadata: it affects scenario-level SNR
   *     selection when link-budget mode is disabled.
   *   - Therefore silent fallback is not allowed.
   */

  if (v == "indoor" || v == "outdoor")
    {
      m_environment = std::move(v);
      return;
    }

  NS_FATAL_ERROR("SetEnvironment: invalid environment string=\""
                 << v
                 << "\". Supported values are: indoor, outdoor.");
}

std::string
PqcExchangeApplication::GetEnvironment() const
{
  /*
   * Contract:
   *   Return the current canonical environment string.
   */

  return m_environment;
}

void
PqcExchangeApplication::SetPacing(std::string v)
{
  /*
   * Contract:
   *   Set the pacing mode from its canonical string representation.
   *
   * Supported values (case-sensitive):
   *     "baseline"
   *     "fixed"
   *     "radioaware"
   *     "radioaware_phase_bounded"
   *
   * Behavior:
   *   - converts the input string to the corresponding PacingMode enum
   *   - aborts on invalid inputs through ParsePacing(...)
   *
   * Important:
   *   - This setter must remain consistent with GetPacing() and ParsePacing().
   *   - No silent fallback is allowed, because pacing mode is part of the
   *     experiment contract and CSV reproducibility metadata.
   */

  m_pacing = ParsePacing(v);
}

PqcExchangeApplication::PacingMode
PqcExchangeApplication::ParsePacing(const std::string& s)
{
  /*
   * Contract:
   *   Parse a string representation of the pacing mode into the corresponding
   *   PacingMode enum value.
   *
   * Supported values (case-sensitive):
   *     "baseline"
   *     "fixed"
   *     "radioaware"
   *     "radioaware_phase_bounded"
   *
   * Behavior:
   *   - returns the corresponding enum for valid inputs
   *   - aborts execution for any unknown string
   *
   * Important:
   *   - This function is the single source of truth for mapping user-facing
   *     pacing strings (CLI / attributes) to internal enum values.
   *   - No implicit fallback is allowed: invalid inputs must fail fast to
   *     preserve reproducibility and avoid silent misconfiguration.
   */

  if (s == "baseline")
    {
      return PacingMode::BASELINE;
    }

  if (s == "fixed")
    {
      return PacingMode::FIXED;
    }

  if (s == "radioaware")
    {
      return PacingMode::RADIOAWARE;
    }

  if (s == "radioaware_phase_bounded")
    {
      return PacingMode::RADIOAWARE_PHASE_BOUNDED;
    }
    
  if (s == "stochastic")
    {
      return PacingMode::STOCHASTIC;
    }

  NS_FATAL_ERROR("ParsePacing: invalid pacing string=\""
                 << s
                 << "\". Supported values are: "
                 << "baseline, fixed, radioaware, radioaware_phase_bounded.");
}

std::string
PqcExchangeApplication::GetPacing() const
{
  /*
   * Contract:
   *   Return the canonical string representation of the current pacing mode.
   *
   * Mapping (bijective with ParsePacing):
   *     PacingMode::BASELINE                  -> "baseline"
   *     PacingMode::FIXED                     -> "fixed"
   *     PacingMode::RADIOAWARE                -> "radioaware"
   *     PacingMode::RADIOAWARE_PHASE_BOUNDED  -> "radioaware_phase_bounded"
   *
   * Responsibilities:
   *   - Provide a stable, human-readable identifier for:
   *       * CSV export (experiment traceability)
   *       * logs / debugging
   *       * reproducibility pipelines
   *
   * Important:
   *   - This function MUST be the exact inverse of ParsePacing(...).
   *   - Returned strings are part of the experiment contract (CSV schema).
   *   - Any change here must be mirrored in ParsePacing(...) and documented.
   *
   * Safety:
   *   - Unknown enum values are not expected in normal operation.
   *   - If encountered, a warning is emitted and "baseline" is returned as a
   *     defensive fallback to avoid breaking downstream pipelines.
   *
   * Rationale:
   *   - Unlike ParsePacing (which must fail fast), this function is used during
   *     export/logging and should not crash long simulation campaigns.
   */

  switch (m_pacing)
    {
    case PacingMode::BASELINE:
      return "baseline";

    case PacingMode::FIXED:
      return "fixed";

    case PacingMode::RADIOAWARE:
      return "radioaware";

    case PacingMode::RADIOAWARE_PHASE_BOUNDED:
      return "radioaware_phase_bounded";
      
    case PacingMode::STOCHASTIC:
      return "stochastic";
    }

  /*
   * Defensive fallback: should never happen unless enum is corrupted or
   * extended without updating this function.
   */
  NS_LOG_WARN("GET_PACING"
              << " unknownEnumValue="
              << static_cast<uint32_t>(m_pacing)
              << " returning=baseline");

  return "baseline";
}

} // namespace ns3
