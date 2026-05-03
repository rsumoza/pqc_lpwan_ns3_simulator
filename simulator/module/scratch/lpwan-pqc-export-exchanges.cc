/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * PQC LoRaWAN Exchange Simulator for ns-3
 *
 * Reproducible experiment driver
 *
 * Contract:
 *   - This file is a thin experiment controller
 *   - The scientific primitive is:
 *         PqcExchangeApplication::RunOneExchange(...)
 *   - CSV output is a 1:1 serialization of ExchangeResult
 *
 * IMPORTANT:
 *   - This driver MUST NOT:
 *       * recompute metrics
 *       * duplicate model state
 *       * introduce derived values
 *   - All metrics must originate from ExchangeResult
 */

#include "ns3/core-module.h"
#include "ns3/lpwan-aloha-collision-model.h"
#include "ns3/lpwan-pqc-helper.h"
#include "ns3/pqc-exchange-application.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace ns3;

// ---------------------------------------------------------------------------
// CSV header (must match ExchangeResult exactly)
// ---------------------------------------------------------------------------
static void
WriteHeader(std::ofstream& out)
{
  out <<
      "environment,sf,variant,"

      "snr_db,rx_dbm,margin_db,noise_dbm,pathloss_db,"

      "payload_bytes_config,"
      "payload_bytes_effective,"
      "g2g_overhead_bytes,phy_payload_bytes_frag_avg,"

      "pk_fragments,ct_fragments,fragment_gap_ms,n_fragments_exchange,"

      "ok_frags,fail_frags,per_observed,"

      "tx_airtime_ms_total,airtime_ms_frag_avg,exchange_latency_s,"

      "energy_j_exchange,"
      "energy_j_exchange_tx,energy_j_exchange_rx,"
      "airtime_energy_tx,airtime_energy_rx,"
      "energy_per_fragment_tx,energy_per_fragment_rx,"

      "lambda_eff,is_on,"
      "attempts_total,attempts_on,attempts_off,"
      "attempts_off_fraction,lambda_eff_attempt_avg,"

      "per_fragment_avg,"
      "per_collision_sim_avg,"
      "per_collision_th_avg,"
      "per_channel_avg,per_total_avg,"

      "pacing_mode,"
      "sf_aware_pacing_threshold,"
      "retry_margin_db,"
      "rx_duty_cycle_enabled,"
      "rx_window_ms,"
      "rx_idle_ms,"
      "ack_batch_size,"

      "data_frames_tx,"
      "ack_frames_tx,"
      "frames_tx_total,"
      "ack_airtime_ms,"

      "ack_logical_failures,"
      "lost_ack_sleep,"
      "lost_ack_channel,"
      "ack_batch_retx_rounds,"
      "ack_per_channel_avg,"
      "ack_per_collision_avg,"
      "ack_per_total_avg,"

      "lost_rx_sleep_data,"
      "lost_rx_sleep_ack,"

      "per_exchange_fail,"
      "retransmissions_count\n";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int
main(int argc, char** argv)
{
  /*
   * CLI PARAMETERS
   *
   * These control the experimental campaign but do NOT affect how results
   * are computed. They are passed as attributes to the application.
   */

  std::string environment = "indoor";
  uint32_t sf = 7;
  std::string variant = "baseline";

  double simTimeS = 10.0;
  std::string outCsv = "lpwan_pqc_sim_exchanges.csv";

  uint32_t periodMs = 500;

  uint32_t payloadBytes = 51;
  uint32_t g2gOverheadBytes = 28;

  std::string pacing = "baseline";

  uint32_t sfAwarePacingThreshold = 7;
  double retryMarginDb = 1.0;
  double gapMinMs = 0.0;
  double gapMaxMs = 0.0;

  bool rxDutyCycleEnabled = false;
  double rxWindowMs = 0.0;
  double rxIdleMs = 0.0;
  uint32_t ackBatchSize = 1;

  double lambdaTotal = 0.0;
  uint32_t logicalChannels = 64;

  uint32_t seed = 1;
  uint32_t run = 1;

  double snrIndoorDb = -3.0;
  double snrOutdoorDb = 8.0;

  uint32_t bwHz = 125000;
  
  bool burstyEnabled = false;
  double onMeanMs = 200.0;
  double offMeanMs = 800.0;
  double phaseOffsetMs = 0.0;
  bool useExplicitEventCollisions = false;
  double interfererAirtimeMs = 200.0;
  std::string burstyModel = "onoff-exp";

  CommandLine cmd(__FILE__);

  cmd.AddValue("burstyModel", "Bursty model (onoff-exp | onoff-square)", burstyModel);
  cmd.AddValue("environment", "indoor/outdoor", environment);
  cmd.AddValue("sf", "Spreading factor", sf);
  cmd.AddValue("variant", "Experiment label", variant);

  cmd.AddValue("simTimeS", "Simulation time (s)", simTimeS);
  cmd.AddValue("outCsv", "Output CSV path", outCsv);

  cmd.AddValue("periodMs", "Exchange period (ms)", periodMs);

  cmd.AddValue("payloadBytes", "Payload bytes per fragment", payloadBytes);
  cmd.AddValue("g2gOverheadBytes", "G2G overhead", g2gOverheadBytes);

  cmd.AddValue("pacing", "Pacing mode", pacing);
  cmd.AddValue("sfAwarePacingThreshold", "SF threshold", sfAwarePacingThreshold);
  cmd.AddValue("retryMarginDb", "Retry margin", retryMarginDb);
  cmd.AddValue("gapMinMs", "Minimum bounded pacing gap (ms)", gapMinMs);
  cmd.AddValue("gapMaxMs", "Maximum bounded pacing gap (ms)", gapMaxMs);

  cmd.AddValue("rxDutyCycleEnabled", "Enable RX duty cycle", rxDutyCycleEnabled);
  cmd.AddValue("rxWindowMs", "RX window (ms)", rxWindowMs);
  cmd.AddValue("rxIdleMs", "RX idle (ms)", rxIdleMs);
  cmd.AddValue("ackBatchSize", "ACK batch size", ackBatchSize);

  cmd.AddValue("lambdaTotal", "Interference load", lambdaTotal);
  cmd.AddValue("logicalChannels", "Logical channels", logicalChannels);

  cmd.AddValue("snrIndoorDb", "Indoor SNR", snrIndoorDb);
  cmd.AddValue("snrOutdoorDb", "Outdoor SNR", snrOutdoorDb);

  cmd.AddValue("bwHz", "Bandwidth (Hz)", bwHz);

  cmd.AddValue("seed", "RNG seed", seed);
  cmd.AddValue("run", "RNG run", run);
  
  cmd.AddValue("burstyEnabled", "Enable bursty interference", burstyEnabled);
  cmd.AddValue("onMeanMs", "ON duration / mean (ms)", onMeanMs);
  cmd.AddValue("offMeanMs", "OFF duration / mean (ms)", offMeanMs);
  cmd.AddValue("phaseOffsetMs", "Square-wave phase offset (ms)", phaseOffsetMs);
  cmd.AddValue("useExplicitEventCollisions", "Use explicit event collisions", useExplicitEventCollisions);
  cmd.AddValue("interfererAirtimeMs", "Explicit interferer airtime (ms)", interfererAirtimeMs);

  cmd.Parse(argc, argv);

  if (outCsv.empty())
    {
      std::cerr << "outCsv must not be empty\n";
      return 1;
    }

  if (simTimeS < 0.0)
    {
      std::cerr << "simTimeS must be >= 0\n";
      return 1;
    }

  if (periodMs == 0)
    {
      std::cerr << "periodMs must be > 0\n";
      return 1;
    }
    if (logicalChannels == 0)
  {
    std::cerr << "logicalChannels must be >= 1\n";
    return 1;
  }

  if (ackBatchSize == 0)
    {
      std::cerr << "ackBatchSize must be >= 1\n";
      return 1;
    }

  if (burstyEnabled && onMeanMs + offMeanMs <= 0.0)
    {
      std::cerr << "Invalid bursty config: ON+OFF must be > 0\n";
      return 1;
    }

  if (useExplicitEventCollisions && interfererAirtimeMs <= 0.0)
    {
      std::cerr << "interfererAirtimeMs must be > 0 in explicit mode\n";
      return 1;
    }


  // -----------------------------------------------------------------------
  // RNG setup
  // -----------------------------------------------------------------------
  RngSeedManager::SetSeed(seed);
  RngSeedManager::SetRun(run);

  // -----------------------------------------------------------------------
  // Model setup
  // -----------------------------------------------------------------------
  NodeContainer nodes;
  nodes.Create(1);

  LpwanPqcHelper helper;
  helper.SetCollisionAttribute("LambdaTotalAvg", DoubleValue(lambdaTotal));
  helper.SetCollisionAttribute("LogicalChannels", UintegerValue(logicalChannels));
  
  helper.SetCollisionAttribute("BurstyEnabled", BooleanValue(burstyEnabled));
  helper.SetCollisionAttribute("OnMeanMs", DoubleValue(onMeanMs));
  helper.SetCollisionAttribute("OffMeanMs", DoubleValue(offMeanMs));
  helper.SetCollisionAttribute("PhaseOffsetMs", DoubleValue(phaseOffsetMs));
  helper.SetCollisionAttribute("UseExplicitEventCollisions", BooleanValue(useExplicitEventCollisions));
  helper.SetCollisionAttribute("InterfererAirtimeMs", DoubleValue(interfererAirtimeMs));
  helper.SetCollisionAttribute("BurstyModel", StringValue(burstyModel));

  ApplicationContainer apps = helper.Install(nodes);
  Ptr<PqcExchangeApplication> app =
      apps.Get(0)->GetObject<PqcExchangeApplication>();

  // -----------------------------------------------------------------------
  // Application configuration
  // -----------------------------------------------------------------------
  app->SetAttribute("Environment", StringValue(environment));
  app->SetAttribute("Sf", UintegerValue(sf));
  app->SetAttribute("Variant", StringValue(variant));

  app->SetAttribute("PayloadBytes", UintegerValue(payloadBytes));
  app->SetAttribute("G2gOverheadBytes", UintegerValue(g2gOverheadBytes));

  app->SetAttribute("Pacing", StringValue(pacing));
  app->SetAttribute("SfAwarePacingThreshold", UintegerValue(sfAwarePacingThreshold));
  app->SetAttribute("RetryMarginDb", DoubleValue(retryMarginDb));
  app->SetAttribute("GapMinMs", UintegerValue(static_cast<uint32_t>(gapMinMs)));
  app->SetAttribute("GapMaxMs", UintegerValue(static_cast<uint32_t>(gapMaxMs)));

  app->SetAttribute("RxDutyCycleEnabled", BooleanValue(rxDutyCycleEnabled));
  app->SetAttribute("RxWindowMs", DoubleValue(rxWindowMs));
  app->SetAttribute("RxIdleMs", DoubleValue(rxIdleMs));
  app->SetAttribute("AckBatchSize", UintegerValue(ackBatchSize));

  app->SetAttribute("SnrIndoorDb", DoubleValue(snrIndoorDb));
  app->SetAttribute("SnrOutdoorDb", DoubleValue(snrOutdoorDb));

  app->SetAttribute("BwHz", UintegerValue(bwHz));

  // -----------------------------------------------------------------------
  // CSV output
  // -----------------------------------------------------------------------
  std::ofstream out(outCsv, std::ios::out | std::ios::trunc);
  if (!out.is_open())
    {
      std::cerr << "Cannot open output file\n";
      return 1;
    }

  WriteHeader(out);

  // -----------------------------------------------------------------------
  // Main experiment loop
  // -----------------------------------------------------------------------
  const uint64_t nExchanges =
      static_cast<uint64_t>(std::floor(simTimeS * 1000.0 / periodMs));
      
  if (nExchanges == 0)
  {
    std::cerr << "WARNING: no exchanges will be executed\n";
  }

  double nowS = 0.0;

  for (uint64_t i = 0; i < nExchanges; ++i)
    {
      const auto r = app->RunOneExchange(nowS, logicalChannels);

      // -------------------------------------------------------------------
      // IMPORTANT:
      //   This is a pure serialization of ExchangeResult
      // -------------------------------------------------------------------

          out << r.environment << ","
          << static_cast<uint32_t>(r.sf) << ","
          << r.variant << ","

          << r.snrDb << ","
          << r.rxDbm << ","
          << r.marginDb << ","
          << r.noiseDbm << ","
          << r.pathLossDb << ","

          << r.payloadBytesConfig << ","
          << r.payloadBytesEffective << ","
          << r.g2gOverheadBytes << ","
          << r.phyPayloadBytesFragAvg << ","

          << r.pkFragments << ","
          << r.ctFragments << ","
          << r.fragGapMs << ","
          << r.nFragmentsExchange << ","

          << r.okFrags << ","
          << r.failFrags << ","
          << r.perObserved << ","

          << r.txAirtimeMsTotal << ","
          << r.airtimeMsFragAvg << ","
          << r.exchangeLatencyS << ","

          << r.energyJExchange << ","
          << r.energyJExchangeTx << ","
          << r.energyJExchangeRx << ","
          << r.airtimeEnergyTx << ","
          << r.airtimeEnergyRx << ","
          << r.energyPerFragmentTx << ","
          << r.energyPerFragmentRx << ","

          << r.lambdaEff << ","
          << r.isOn << ","
          << r.attemptsTotal << ","
          << r.attemptsOn << ","
          << r.attemptsOff << ","
          << r.attemptsOffFraction << ","
          << r.lambdaEffAttemptAvg << ","

          << r.perFragmentAvg << ","
          << r.perCollisionSimAvg << ","
          << r.perCollisionThAvg << ","
          << r.perChannelAvg << ","
          << r.perTotalAvg << ","

          << r.pacingMode << ","
          << r.sfAwarePacingThreshold << ","
          << r.retryMarginDb << ","
          << (r.rxDutyCycleEnabled ? 1 : 0) << ","
          << r.rxWindowMs << ","
          << r.rxIdleMs << ","
          << r.ackBatchSize << ","

          << r.dataFramesTx << ","
          << r.ackFramesTx << ","
          << r.framesTxTotal << ","
          << r.ackAirtimeMs << ","

          << r.ackLogicalFailures << ","
          << r.lostAckSleep << ","
          << r.lostAckChannel << ","
          << r.ackBatchRetxRounds << ","
          << r.ackPerChannelAvg << ","
          << r.ackPerCollisionAvg << ","
          << r.ackPerTotalAvg << ","

          << r.lostRxSleepData << ","
          << r.lostRxSleepAck << ","

          << (r.perExchangeFail ? 1 : 0) << ","
          << r.retransmissionsCount
          << "\n";

      nowS += static_cast<double>(periodMs) / 1000.0;
    }

  out.close();
  return 0;
}
