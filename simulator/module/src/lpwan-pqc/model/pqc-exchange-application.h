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

#ifndef NS3_PQC_EXCHANGE_APPLICATION_H
#define NS3_PQC_EXCHANGE_APPLICATION_H

#include "ns3/application.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"
#include "ns3/type-id.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

static constexpr uint32_t LORAWAN_ACK_PHY_BYTES = 12;

class LpwanAlohaCollisionModel;
class LpwanPacketErrorModel;

/**
 * \ingroup lpwan-pqc
 * \brief Application that simulates one PQC exchange over a LoRa-like PHY.
 *
 * Contract:
 *   - exchange-driven simulation
 *   - one call to RunOneExchange() returns one ExchangeResult
 *   - ExchangeResult is the single source of truth for CSV export
 */
class PqcExchangeApplication : public Application
{
public:
  /**
   * \brief Pacing policies supported by the simulator.
   */
  enum class PacingMode
  {
    /**
     * \brief No additional pacing beyond natural protocol sequencing.
     */
    BASELINE,

    /**
     * \brief Constant fixed inter-attempt / inter-fragment gap.
     */
    FIXED,

    /**
     * \brief Radio-aware pacing derived from LoRa symbol time and margins.
     */
    RADIOAWARE,

    /**
     * \brief Radio-aware pacing with bounded gaps and RX-phase alignment.
     */
    RADIOAWARE_PHASE_BOUNDED,
    
    STOCHASTIC
  };

  /**
   * \brief Final per-exchange export record.
   *
   * This structure is the single source of truth for CSV export.
   * Every CSV row must be a 1:1 serialization of exactly one ExchangeResult.
   */
  struct ExchangeResult
  {
    // ----------------------------------------------------------------------
    // CSV identity / experiment labels
    // ----------------------------------------------------------------------
    std::string environment;
    uint8_t sf = 7;
    std::string variant;
    std::string pacingMode;

    // ----------------------------------------------------------------------
    // Scenario / link diagnostics
    // ----------------------------------------------------------------------
    double snrDb = 0.0;
    double rxDbm = 0.0;
    double marginDb = 0.0;
    double noiseDbm = 0.0;
    double pathLossDb = 0.0;

    // ----------------------------------------------------------------------
    // Fragmentation / payload contract
    // ----------------------------------------------------------------------
    uint32_t payloadBytesConfig = 0;
    uint32_t payloadBytesEffective = 0;
    uint32_t g2gOverheadBytes = 0;
    double phyPayloadBytesFragAvg = 0.0;

    uint32_t pkFragments = 0;
    uint32_t ctFragments = 0;
    double fragGapMs = 0.0;
    uint32_t nFragmentsExchange = 0;

    // ----------------------------------------------------------------------
    // Observed fragment outcomes
    // ----------------------------------------------------------------------
    uint32_t okFrags = 0;
    uint32_t failFrags = 0;

    /**
     * \brief Observed fragment error rate for the exchange.
     *
     * Defined as:
     *
     *   failFrags / nFragmentsExchange
     *
     * with defensive 0 when nFragmentsExchange == 0.
     */
    double perObserved = 0.0;

    // ----------------------------------------------------------------------
    // Airtime / latency
    // ----------------------------------------------------------------------
    double txAirtimeMsTotal = 0.0;
    double airtimeMsFragAvg = 0.0;
    double exchangeLatencyS = 0.0;

    // ----------------------------------------------------------------------
    // Energy
    // ----------------------------------------------------------------------
    double energyJExchange = 0.0;
    double energyJExchangeTx = 0.0;
    double energyJExchangeRx = 0.0;
    double airtimeEnergyTx = 0.0;
    double airtimeEnergyRx = 0.0;
    double energyPerFragmentTx = 0.0;
    double energyPerFragmentRx = 0.0;

    // ----------------------------------------------------------------------
    // Bursty / phase-aware diagnostics
    // ----------------------------------------------------------------------
    double lambdaEff = 0.0;
    uint32_t isOn = 0;
    uint32_t attemptsTotal = 0;
    uint32_t attemptsOn = 0;
    uint32_t attemptsOff = 0;
    double attemptsOffFraction = 0.0;
    double lambdaEffAttemptAvg = 0.0;

    // ----------------------------------------------------------------------
    // Nominal probabilities
    // ----------------------------------------------------------------------
    double perFragmentAvg = 0.0;
    double perCollisionSimAvg = 0.0;
    double perCollisionThAvg = 0.0;
    double perChannelAvg = 0.0;
    double perTotalAvg = 0.0;

    // ----------------------------------------------------------------------
    // Exported pacing / retry / RX config snapshot
    // ----------------------------------------------------------------------
    uint32_t sfAwarePacingThreshold = 255;
    double retryMarginDb = 1.0;

    bool rxDutyCycleEnabled = false;
    double rxWindowMs = 0.0;
    double rxIdleMs = 0.0;

    // ----------------------------------------------------------------------
    // ACK-aware contract
    // ----------------------------------------------------------------------
    uint32_t ackBatchSize = 1;
    uint32_t dataFramesTx = 0;
    uint32_t ackFramesTx = 0;
    uint32_t framesTxTotal = 0;
    double ackAirtimeMs = 0.0;

    uint32_t ackLogicalFailures = 0;
    uint32_t lostAckSleep = 0;
    uint32_t lostAckChannel = 0;
    uint32_t ackBatchRetxRounds = 0;

    double ackPerChannelAvg = 0.0;
    double ackPerCollisionAvg = 0.0;
    double ackPerTotalAvg = 0.0;

    // ----------------------------------------------------------------------
    // RX sleep counters
    // ----------------------------------------------------------------------
    uint32_t lostRxSleepData = 0;
    uint32_t lostRxSleepAck = 0;

    // ----------------------------------------------------------------------
    // Exchange terminal status
    // ----------------------------------------------------------------------
    bool perExchangeFail = false;
    uint32_t retransmissionsCount = 0;
  };

  static TypeId GetTypeId();

  PqcExchangeApplication();
  ~PqcExchangeApplication() override;

  void SetCollisionModel(Ptr<LpwanAlohaCollisionModel> model);
  void SetPacketErrorModel(Ptr<LpwanPacketErrorModel> model);

  /**
   * \brief Execute one full logical PQC exchange and return its result.
   *
   * \param nowS Absolute exchange start time in seconds.
   * \param logicalChannels Number of logical channels exposed to the collision model.
   * \return One fully populated ExchangeResult.
   */
  ExchangeResult RunOneExchange(double nowS, uint32_t logicalChannels);

  void SetVariant(std::string v);
  std::string GetVariant() const;

  void SetEnvironment(std::string v);
  std::string GetEnvironment() const;

  void SetPacing(std::string v);
  std::string GetPacing() const;
  static PacingMode ParsePacing(const std::string& s);
  

private:
  /**
   * \brief One DATA fragment that has been delivered at DATA level and is
   *        pending logical acknowledgment.
   *
   * This structure is used by the ACK batching subsystem. A fragment may be
   * successfully delivered at DATA level, yet still fail at exchange level if
   * the corresponding ACK batch is never successfully received.
   */
  struct PendingAckEntry
  {
    /**
     * \brief Logical fragment index inside the current exchange.
     */
    uint32_t fragIndex = 0;

    /**
     * \brief DATA frame airtime in milliseconds for this fragment.
     */
    double airtimeFragMs = 0.0;

    /**
     * \brief Nominal PHY/channel failure probability for this fragment.
     *
     * This models PHY/channel decoding failures only, not collisions.
     */
    double pPhy = 0.0;

    /**
     * \brief Number of retransmissions already consumed by this fragment.
     *
     * 0 means first DATA attempt not retried yet.
     */
    uint32_t retxUsed = 0;
  };

  /**
   * \brief Mutable runtime state of one logical exchange.
   *
   * This structure contains only execution-time state needed to advance the
   * protocol timeline and fragment sequence.
   */
  struct ExchangeRuntimeState
  {
    /**
     * \brief Absolute start time of the logical exchange in seconds.
     */
    double exchangeStartS = 0.0;

    /**
     * \brief Mutable protocol clock in seconds.
     *
     * This timestamp advances as fragment airtime, pacing gaps, retries, ACKs,
     * and recovery rounds are executed.
     */
    double nowS = 0.0;

    /**
     * \brief Number of logical channels visible to the collision model.
     */
    uint32_t logicalChannels = 0;

    /**
     * \brief Per-fragment crypto payload plan for the full exchange.
     *
     * This vector represents the logical sequence of PK + CT fragments.
     */
    std::vector<uint32_t> fragPayloads;

    /**
     * \brief Total number of logical fragments in the exchange.
     */
    uint32_t nFrags = 0;
  };

  /**
   * \brief Logical exchange outcome state.
   *
   * These counters represent logical outcomes at exchange level, not merely
   * physical transmissions.
   */
  struct ExchangeOutcomeState
  {
    /**
     * \brief Number of fragments that are successful at exchange level.
     *
     * A fragment becomes successful only after successful logical ACK handling.
     */
    uint32_t okFrags = 0;

    /**
     * \brief Number of fragments that fail at exchange level.
     *
     * This includes:
     *   - terminal DATA failures,
     *   - fragments pending ACK when the exchange aborts,
     *   - fragments never started due to early exchange termination.
     */
    uint32_t failFrags = 0;

    /**
     * \brief Number of retransmissions actually consumed.
     *
     * Counts only retries beyond the first transmission attempt.
     */
    uint32_t totalRetx = 0;

    /**
     * \brief Number of DATA frames actually transmitted on air.
     *
     * Counts:
     *   - the first attempt of each actually attempted fragment,
     *   - every retransmission attempt.
     *
     * This is the authoritative source for CSV field data_frames_tx.
     * It must not be derived from nFrags + totalRetx because an exchange may
     * abort before all logical fragments are attempted.
     */
    uint32_t dataFramesTxReal = 0;

    /**
     * \brief True iff the exchange has already entered terminal failure state.
     */
    bool exchangeFailed = false;
  };

  /**
   * \brief Accumulated statistics and diagnostics for one exchange.
   *
   * These accumulators feed exported averages and summary metrics.
   */
  struct ExchangeStatsState
  {
    /**
     * \brief Sum of nominal PHY failure probabilities across logical fragments.
     */
    double sumPhy = 0.0;

    /**
     * \brief Sum of simulated collision probabilities across logical fragments.
     */
    double sumColSim = 0.0;

    /**
     * \brief Sum of theoretical/analytical collision probabilities.
     */
    double sumColTh = 0.0;

    /**
     * \brief Sum of total failure probabilities across logical fragments.
     */
    double sumTot = 0.0;

    /**
     * \brief Sum of PHY payload bytes across logical fragments.
     */
    double sumPhyPayloadBytes = 0.0;

    /**
     * \brief Sum of all transmitted airtime (DATA + ACK) in milliseconds.
     */
    double txAirtimeMsSum = 0.0;

    /**
     * \brief End-to-end elapsed exchange time in milliseconds.
     *
     * Includes airtime and all pacing/recovery gaps.
     */
    double elapsedMsSum = 0.0;

    /**
     * \brief Sum of per-fragment DATA airtime in milliseconds.
     */
    double airtimeFragMsSum = 0.0;

    /**
     * \brief Total number of DATA attempts considered by burst/load diagnostics.
     */
    uint32_t attemptsTotal = 0;

    /**
     * \brief Number of attempts that occurred during ON burst state.
     */
    uint32_t attemptsOn = 0;

    /**
     * \brief Number of attempts that occurred during OFF burst state.
     */
    uint32_t attemptsOff = 0;

    /**
     * \brief Sum of effective interferer rate samples over DATA attempts.
     */
    double lambdaEffSum = 0.0;

    /**
     * \brief Number of DATA attempts lost due to RX sleep.
     */
    uint32_t lostRxSleepData = 0;

    /**
     * \brief Number of ACK attempts lost due to RX sleep.
     */
    uint32_t lostRxSleepAck = 0;

    /**
     * \brief Estimated receiver active listening time in milliseconds.
     */
    double rxActiveMsReal = 0.0;
  };
    
  /**
   * \brief ACK subsystem state for one exchange.
   *
   * This structure contains logical ACK batching state, ACK diagnostics, and
   * ACK-related counters.
   */
  struct ExchangeAckState
  {
    /**
     * \brief Queue of DATA-delivered fragments still pending logical ACK.
     */
    std::vector<PendingAckEntry> pendingAckQueue;

    /**
     * \brief Number of ACK frames actually transmitted on air.
     */
    uint32_t ackFramesTx = 0;

    /**
     * \brief Sum of ACK airtime in milliseconds.
     */
    double ackAirtimeMsSum = 0.0;

    /**
     * \brief Number of logical fragments affected by ACK failures.
     */
    uint32_t ackLogicalFailures = 0;

    /**
     * \brief Number of ACK frames lost because the receiver was asleep.
     */
    uint32_t lostAckSleep = 0;

    /**
     * \brief Number of ACK frames lost due to PHY/collision failure.
     */
    uint32_t lostAckChannel = 0;

    /**
     * \brief Number of ACK recovery rounds executed.
     */
    uint32_t ackBatchRetxRounds = 0;

    /**
     * \brief Sum of ACK PHY/channel failure probabilities.
     */
    double ackPerChannelSum = 0.0;

    /**
     * \brief Sum of ACK collision failure probabilities.
     */
    double ackPerCollisionSum = 0.0;

    /**
     * \brief Sum of ACK total failure probabilities.
     */
    double ackPerTotalSum = 0.0;

    /**
     * \brief Number of ACK attempts contributing to ACK averages.
     */
    uint32_t ackAttemptsTotal = 0;
  };

  /**
   * \brief Complete mutable execution context of one logical exchange.
   *
   * This structure groups:
   *   - runtime protocol state,
   *   - logical outcome counters,
   *   - accumulated statistics,
   *   - ACK subsystem state,
   *   - exported ExchangeResult snapshot.
   *
   * The goal is to keep responsibilities separated while still passing a
   * single context object through the exchange execution pipeline.
   */
  struct ExchangeContext
  {
    ExchangeRuntimeState runtime;
    ExchangeOutcomeState outcome;
    ExchangeStatsState stats;
    ExchangeAckState ack;
    ExchangeResult out;
  };

  /**
   * \brief Context used to compute inter-fragment pacing.
   */
  struct FragmentGapContext
  {
    uint8_t sf = 7;
    double bandwidthHz = 125000.0;
    double marginDb = 0.0;
    PacingMode pacingMode = PacingMode::BASELINE;

    double fixedGapMs = 0.0;
    double gapMinMs = 0.0;
    double gapMaxMs = 0.0;

    double kTsym = 4.0;
    uint8_t sfAwarePacingThreshold = 255;

    double marginRefDb = 12.0;
    double marginAlpha = 0.10;
  };

  /**
   * \brief Context used to compute DATA retry pacing.
   */
  struct DataAttemptGapContext
{
  uint8_t sf = 7;
  double bandwidthHz = 125000.0;
  double retryMarginDb = 1.0;
  uint32_t attemptIndex = 0; // 1 = first retry
  PacingMode pacingMode = PacingMode::BASELINE;

  double fixedGapMs = 0.0;
  double gapMinMs = 0.0;
  double gapMaxMs = 0.0;

  double kTsym = 4.0;
  uint8_t sfAwarePacingThreshold = 255;

  double retryMarginRefDb = 12.0;
  double retryAlpha = 0.15;
  double retryBackoffGamma = 0.25;

  bool rxDutyCycleEnabled = false;
  double rxWindowMs = 0.0;
  double rxIdleMs = 0.0;
};

  /**
   * \brief Context used to compute ACK recovery pacing.
   */
  struct AckRecoveryGapContext
{
  uint8_t sf = 7;
  double bandwidthHz = 125000.0;
  double ackRecoveryMarginDb = 12.0;
  uint32_t recoveryRound = 0; // 1 = first ACK recovery round
  PacingMode pacingMode = PacingMode::BASELINE;

  double fixedGapMs = 0.0;
  double gapMinMs = 0.0;
  double gapMaxMs = 0.0;

  double kTsym = 4.0;
  uint8_t sfAwarePacingThreshold = 255;

  double ackRecoveryMarginRefDb = 12.0;
  double ackRecoveryAlpha = 0.10;
  double ackRecoveryBackoffGamma = 0.20;

  bool rxDutyCycleEnabled = false;
  double rxWindowMs = 0.0;
  double rxIdleMs = 0.0;
};

  ExchangeContext InitializeExchange(double nowS, uint32_t logicalChannels) const;
  void InitializeExchangeMetadata(ExchangeContext& ctx) const;
  void InitializeScenarioDiagnostics(ExchangeContext& ctx) const;
  void FinalizeExchange(ExchangeContext& ctx) const;

  void RunOneFragment(ExchangeContext& ctx, uint32_t fragIndex);
  bool RunFragmentArq(ExchangeContext& ctx, PendingAckEntry& e);
  bool FlushAckBatch(ExchangeContext& ctx, bool force);

  void BuildFragmentPlan(std::vector<uint32_t>& payloadBytesPerFragment) const;
  void BuildFragmentPlan(std::vector<uint32_t>& fragPayloads,
                         uint32_t totalBytes,
                         uint32_t payloadBytes) const;

  uint32_t ComputePhyPayloadBytes(uint32_t payloadCrypto) const;
  double ComputeFragmentAirtimeMs(uint32_t payloadPhy) const;

  double ComputeFragmentGapMs(const FragmentGapContext& ctx) const;
  double ComputeDataAttemptGapMs(const DataAttemptGapContext& ctx) const;
  double ComputeAckRecoveryGapMs(const AckRecoveryGapContext& ctx) const;

  void AccumulateNominalFragmentStats(ExchangeContext& ctx,
                                      uint32_t payloadPhy,
                                      double airtimeFragMs,
                                      double airtimeFragS,
                                      double pPhy);

  void AccumulateAttemptDiagnostics(ExchangeContext& ctx,
                                    double attemptNowS) const;

  double ComputeChannelFailureProbability(uint32_t payloadBytes, double snrDb);
  double ComputeCollisionFailureProbability(double nowS,
                                            double txAirtimeMsTotal,
                                            uint32_t logicalChannels) const;
  double ComputeTotalFailureProbability(double pPhy, double pCol) const;

  double GetScenarioSnrDb() const;

  /**
   * \brief Return true iff the full frame interval [startMs, startMs+durationMs]
   *        is fully contained within an active RX window.
   *
   * This implements full-window containment semantics required by the simulator
   * contract. Partial overlap with an RX window does NOT count as successful
   * reception.
   */
  bool IsFrameFullyContainedInRxWindow(double startMs,
                                       double durationMs,
                                       double rxWindowMs,
                                       double rxIdleMs) const;

  /**
   * \brief Shift \p nowS to a receiver-active instant that can contain a full
   *        frame of duration \p durationMs.
   *
   * This is used by phase-aware pacing under RX duty cycling. If the current
   * phase is already valid, the timestamp is unchanged. Otherwise the timestamp
   * is aligned to the next RX window start.
   */
  double AlignToRxWindowStartForFullContainment(double nowS,
                                                double durationMs,
                                                double rxWindowMs,
                                                double rxIdleMs) const;

  static double GetSnrMinDb(uint32_t sf);
  double GetSensitivityDbm(uint32_t sf, uint32_t bwHz, double noiseFigureDb) const;
  double ComputeNoiseDbm() const;
  double ComputePathLossDb(double dM) const;
  double ComputeSnrDb(double dM) const;
  
  void
  ApplyRetryGap(ExchangeContext& ctx,
                PendingAckEntry& e,
                double attemptEndS,
                const std::string& cause);
                                      
  void
  ApplyAckRecoveryGap(ExchangeContext& ctx,
                      PendingAckEntry& e);
                      
  void ValidateConfiguration() const;

private:
  // Environment configuration
  std::string m_environment{"indoor"};
  uint8_t     m_sf{7};
  std::string m_variant{"baseline"};

  // Fragmentation configuration
  uint32_t m_pkFragments{6};
  uint32_t m_ctFragments{9};
  uint32_t m_payloadBytes{51};

  // Auto fragmentation
  bool     m_autoFragments{false};
  uint32_t m_pkBytes{800};
  uint32_t m_ctBytes{768};
  uint32_t m_maxPayloadOverrideBytes{0};

  // Pacing
  PacingMode m_pacing{PacingMode::BASELINE};
  uint32_t   m_fixedGapMs{0};
  double     m_kTsym{4.0};
  uint32_t   m_sfAwarePacingThreshold{255};

  double m_marginRefDb{12.0};
  double m_marginAlpha{0.10};

  double m_retryMarginDb{1.0};
  double m_retryMarginRefDb{12.0};
  double m_retryAlpha{0.15};
  double m_retryBackoffGamma{0.25};

  double m_ackRecoveryMarginDb{12.0};
  double m_ackRecoveryMarginRefDb{12.0};
  double m_ackRecoveryAlpha{0.10};
  double m_ackRecoveryBackoffGamma{0.20};

  uint32_t m_gapMinMs{0};
  uint32_t m_gapMaxMs{200};

  // Retry limits
  uint32_t m_maxRetries{2};

  // Baseline scenario SNRs
  double m_snrIndoorDb{-3.0};
  double m_snrOutdoorDb{8.0};

  // G2G overhead
  uint32_t m_g2gOverheadBytes{28};

  // Energy
  double m_txEnergyJPerMs{0.0004};
  double m_rxEnergyJPerMs{0.0002};
  bool   m_rxEnergyUsesExchangeLatency{true};

  // LoRa/PER table parameters
  uint32_t m_bwHz{125000};
  uint8_t  m_crDen{8};

  // Receiver duty cycle / ACK
  bool     m_rxDutyCycleEnabled{false};
  double   m_rxWindowMs{0.0};
  double   m_rxIdleMs{0.0};
  uint32_t m_ackBatchSize{1};

  // Link budget parameters
  bool   m_useLinkBudget{false};
  double m_txPowerDbm{14.0};
  double m_distanceM{1000.0};
  double m_refDistanceM{1.0};
  double m_refPathLossDb{32.0};
  double m_pathLossExp{3.5};
  double m_shadowingSigmaDb{0.0};
  double m_noiseFigureDb{6.0};

  // Injected models / RNG / state
  Ptr<LpwanAlohaCollisionModel> m_col;
  Ptr<LpwanPacketErrorModel>    m_per;
  Ptr<UniformRandomVariable>    m_uv;
  Ptr<NormalRandomVariable>     m_shadowRv;
};

} // namespace ns3

#endif // NS3_PQC_EXCHANGE_APPLICATION_H
