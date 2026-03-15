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
 * \file lpwan-pqc-export-exchanges.cc
 *
 * \brief Reproducible scratch driver for the lpwan-pqc module.
 *
 * Runs a sequence of PQC exchanges and exports one CSV row per exchange.
 *
 * Contract:
 *  - the scratch driver is the experiment controller
 *  - PqcExchangeApplication::RunOneExchange(nowS, logicalChannels) is the
 *    scientific primitive
 *  - the CSV is written 1:1 from ExchangeResult
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

/**
 * \brief Write the stable CSV header.
 *
 * Keep this order aligned 1:1 with ExchangeResult export.
 */
static void
WriteHeader(std::ofstream& out)
{
  out <<
      "environment,sf,variant,validation_mode,"

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

int
main(int argc, char** argv)
{
  // ---------------------------------------------------------------------------
  // CLI defaults
  // ---------------------------------------------------------------------------

  // Scenario / labels
  std::string environment = "indoor";
  uint32_t sf = 7;
  std::string variant = "baseline";

  // Simulation control
  double simTimeS = 10.0;
  std::string outCsv = "lpwan_pqc_sim_exchanges.csv";
  uint32_t periodMs = 500; // scratch-only time-base control
  std::string timeBase = "start_to_start";
  double idleAfterExchangeMs = 0.0;

  // Fragmentation
  uint32_t payloadBytes = 51;
  uint32_t g2gOverheadBytes = 28;

  uint32_t pkFragments = 6;
  uint32_t ctFragments = 9;

  bool autoFragments = false;
  uint32_t pkBytes = 800;
  uint32_t ctBytes = 768;
  uint32_t maxPayloadOverrideBytes = 0;

  // Pacing
  std::string pacing = "baseline";
  uint32_t fixedGapMs = 0;
  double kTsym = 2.0;
  double gapMinMs = 0.0;
  double gapMaxMs = 50.0;
  uint32_t sfAwarePacingThreshold = 255;
  double retryMarginDb = 1.0;

  // Retries / RX / ACK
  uint32_t maxRetries = 2;
  bool rxDutyCycleEnabled = false;
  double rxWindowMs = 0.0;
  double rxIdleMs = 0.0;
  uint32_t ackBatchSize = 1;

  // Collision model
  double lambdaTotal = 0.0;
  uint32_t logicalChannels = 64;

  bool burstyEnabled = false;
  std::string burstyModel = "onoff-exp";
  double onMeanMs = 200.0;
  double offMeanMs = 800.0;
  double phaseOffsetMs = 0.0;

  bool useExplicitEventCollisions = false;

  // RNG
  uint32_t seed = 1;
  uint32_t run = 1;

  // Scenario SNR
  double snrIndoorDb = -3.0;
  double snrOutdoorDb = 8.0;

  // Link budget
  bool useLinkBudget = false;
  double txPowerDbm = 14.0;
  double distanceM = 1000.0;
  double refDistanceM = 1.0;
  double refPathLossDb = 32.0;
  double pathLossExp = 3.5;
  double shadowingSigmaDb = 0.0;
  double noiseFigureDb = 6.0;

  // PER table
  bool usePerTable = false;
  std::string perTablePath = "";
  uint32_t bwHz = 125000;
  uint32_t crDen = 8;
  double cfoLambda = 0.0;
  bool validationMode = false;

  CommandLine cmd(__FILE__);

  // Scenario / labels
  cmd.AddValue("environment", "indoor/outdoor", environment);
  cmd.AddValue("sf", "Spreading factor [7..12]", sf);
  cmd.AddValue("variant", "Experiment label", variant);

  // Simulation control
  cmd.AddValue("simTimeS", "Simulation time (s)", simTimeS);
  cmd.AddValue("outCsv", "Output CSV path", outCsv);
  cmd.AddValue("periodMs", "Exchange period (ms) for scratch time base", periodMs);
  cmd.AddValue("timeBase", "start_to_start | end_to_start", timeBase);
  cmd.AddValue("idleAfterExchangeMs",
               "Extra idle after exchange when timeBase=end_to_start (ms)",
               idleAfterExchangeMs);

  // Fragmentation
  cmd.AddValue("pk_fragments", "PK fragments per exchange (manual)", pkFragments);
  cmd.AddValue("ct_fragments", "CT fragments per exchange (manual)", ctFragments);
  cmd.AddValue("payloadBytes", "Payload bytes per fragment (manual)", payloadBytes);
  cmd.AddValue("autoFragments", "Enable SF-aware auto-fragmentation", autoFragments);
  cmd.AddValue("pkBytes", "Total PK bytes (auto)", pkBytes);
  cmd.AddValue("ctBytes", "Total CT bytes (auto)", ctBytes);
  cmd.AddValue("maxPayloadOverrideBytes",
               "Override max payload cap when autoFragments=1 (0=SF-aware; >0 fixed cap)",
               maxPayloadOverrideBytes);
  cmd.AddValue("g2gOverheadBytes", "G2G per-fragment overhead (bytes)", g2gOverheadBytes);

  // Pacing
  cmd.AddValue("pacing", "baseline | fixed | radioaware | radioaware_phase_bounded", pacing);
  cmd.AddValue("fixedGapMs", "Fixed gap (ms) if pacing=fixed", fixedGapMs);
  cmd.AddValue("kTsym", "Radio-aware multiplier: gap ≈ kTsym*Tsym", kTsym);
  cmd.AddValue("gapMinMs", "Minimum bounded gap (ms)", gapMinMs);
  cmd.AddValue("gapMaxMs", "Maximum bounded gap (ms)", gapMaxMs);
  cmd.AddValue("sfAwarePacingThreshold",
               "SF threshold for enabling radio-aware pacing",
               sfAwarePacingThreshold);
  cmd.AddValue("retryMarginDb",
               "Retry margin parameter for retry pacing",
               retryMarginDb);

  // Retries / ACK / RX duty cycle
  cmd.AddValue("maxRetries", "Max retransmissions per fragment", maxRetries);
  cmd.AddValue("rxDutyCycleEnabled", "Enable receiver duty-cycled listening", rxDutyCycleEnabled);
  cmd.AddValue("rxWindowMs", "Receiver active window in ms", rxWindowMs);
  cmd.AddValue("rxIdleMs", "Receiver idle/sleep interval in ms", rxIdleMs);
  cmd.AddValue("ackBatchSize", "ACK every N successful fragments", ackBatchSize);

  // Collision model
  cmd.AddValue("lambdaTotal", "Total interferer attempt rate (attempts/s)", lambdaTotal);
  cmd.AddValue("logicalChannels", "Number of logical channels", logicalChannels);
  cmd.AddValue("burstyEnabled", "Enable ON/OFF bursty interference", burstyEnabled);
  cmd.AddValue("burstyModel", "onoff-exp | onoff-square", burstyModel);
  cmd.AddValue("onMeanMs", "Mean/nominal ON duration (ms)", onMeanMs);
  cmd.AddValue("offMeanMs", "Mean/nominal OFF duration (ms)", offMeanMs);
  cmd.AddValue("phaseOffsetMs", "Phase offset for onoff-square bursty model (ms)", phaseOffsetMs);
  cmd.AddValue("useExplicitEventCollisions",
               "Enable explicit event-driven collision schedule",
               useExplicitEventCollisions);

  // Scenario SNR / link budget
  cmd.AddValue("snrIndoorDb", "Baseline SNR for indoor scenario (dB)", snrIndoorDb);
  cmd.AddValue("snrOutdoorDb", "Baseline SNR for outdoor scenario (dB)", snrOutdoorDb);
  cmd.AddValue("useLinkBudget", "If true, compute SNR from link budget", useLinkBudget);
  cmd.AddValue("txPowerDbm", "Tx power (dBm) for link budget", txPowerDbm);
  cmd.AddValue("distanceM", "Tx-Rx distance (m) for link budget", distanceM);
  cmd.AddValue("refDistanceM", "Reference distance d0 (m)", refDistanceM);
  cmd.AddValue("refPathLossDb", "Path loss at d0 (dB)", refPathLossDb);
  cmd.AddValue("pathLossExp", "Log-distance exponent n", pathLossExp);
  cmd.AddValue("shadowingSigmaDb", "Shadowing sigma (dB), 0 disables", shadowingSigmaDb);
  cmd.AddValue("noiseFigureDb", "Receiver noise figure (dB)", noiseFigureDb);

  // PER table
  cmd.AddValue("usePerTable", "If true, use PER table CSV instead of analytic model", usePerTable);
  cmd.AddValue("perTablePath", "PER table CSV path", perTablePath);
  cmd.AddValue("bwHz", "Bandwidth key for PER table (Hz)", bwHz);
  cmd.AddValue("crDen", "CR denominator for CR=4/CrDen", crDen);
  cmd.AddValue("cfoLambda", "CFO lambda key for PER table", cfoLambda);
  cmd.AddValue("validationMode",
               "If true, force payloadPhy = MaxPayloadBytes(SF) for PER-table validation",
               validationMode);

  // RNG
  cmd.AddValue("seed", "RNG seed", seed);
  cmd.AddValue("run", "RNG run", run);

  cmd.Parse(argc, argv);

  // ---------------------------------------------------------------------------
  // RNG configuration
  // ---------------------------------------------------------------------------
  RngSeedManager::SetSeed(seed);
  RngSeedManager::SetRun(run);

  // ---------------------------------------------------------------------------
  // Minimal ns-3 scaffolding
  // ---------------------------------------------------------------------------
  NodeContainer nodes;
  nodes.Create(1);

  LpwanPqcHelper h;

  // Collision model configuration through helper
  h.SetCollisionAttribute("LambdaTotalAvg", DoubleValue(lambdaTotal));
  h.SetCollisionAttribute("LogicalChannels", UintegerValue(logicalChannels));
  h.SetCollisionAttribute("BurstyEnabled", BooleanValue(burstyEnabled));
  h.SetCollisionAttribute("BurstyModel", StringValue(burstyModel));
  h.SetCollisionAttribute("OnMeanMs", DoubleValue(onMeanMs));
  h.SetCollisionAttribute("OffMeanMs", DoubleValue(offMeanMs));
  h.SetCollisionAttribute("PhaseOffsetMs", DoubleValue(phaseOffsetMs));

  ApplicationContainer apps = h.Install(nodes);
  Ptr<PqcExchangeApplication> app = apps.Get(0)->GetObject<PqcExchangeApplication>();

  Ptr<LpwanAlohaCollisionModel> col = h.GetCollisionModel();
  if (col)
    {
      col->SetAttribute("UseExplicitEventCollisions",
                        BooleanValue(useExplicitEventCollisions));

      if (useExplicitEventCollisions)
        {
          // Conservative horizon so the explicit event schedule covers the campaign.
          col->BuildEventSchedule(simTimeS + 10.0);
        }
    }

  // ---------------------------------------------------------------------------
  // Application attributes
  // ---------------------------------------------------------------------------
  app->SetAttribute("Environment", StringValue(environment));
  app->SetAttribute("Sf", UintegerValue(sf));
  app->SetAttribute("Variant", StringValue(variant));

  app->SetAttribute("PkFragments", UintegerValue(pkFragments));
  app->SetAttribute("CtFragments", UintegerValue(ctFragments));
  app->SetAttribute("PayloadBytes", UintegerValue(payloadBytes));

  app->SetAttribute("AutoFragments", BooleanValue(autoFragments));
  app->SetAttribute("PkBytes", UintegerValue(pkBytes));
  app->SetAttribute("CtBytes", UintegerValue(ctBytes));
  app->SetAttribute("MaxPayloadOverrideBytes", UintegerValue(maxPayloadOverrideBytes));

  app->SetAttribute("Pacing", StringValue(pacing));
  app->SetAttribute("FixedGapMs", UintegerValue(fixedGapMs));
  app->SetAttribute("KTsym", DoubleValue(kTsym));
  app->SetAttribute("GapMinMs",
                    UintegerValue(static_cast<uint32_t>(std::max(0.0, gapMinMs))));
  app->SetAttribute("GapMaxMs",
                    UintegerValue(static_cast<uint32_t>(std::max(0.0, gapMaxMs))));
  app->SetAttribute("SfAwarePacingThreshold", UintegerValue(sfAwarePacingThreshold));
  app->SetAttribute("RetryMarginDb", DoubleValue(retryMarginDb));

  app->SetAttribute("MaxRetries", UintegerValue(maxRetries));

  app->SetAttribute("SnrIndoorDb", DoubleValue(snrIndoorDb));
  app->SetAttribute("SnrOutdoorDb", DoubleValue(snrOutdoorDb));

  app->SetAttribute("G2gOverheadBytes", UintegerValue(g2gOverheadBytes));

  app->SetAttribute("UseLinkBudget", BooleanValue(useLinkBudget));
  app->SetAttribute("TxPowerDbm", DoubleValue(txPowerDbm));
  app->SetAttribute("DistanceM", DoubleValue(distanceM));
  app->SetAttribute("RefDistanceM", DoubleValue(refDistanceM));
  app->SetAttribute("RefPathLossDb", DoubleValue(refPathLossDb));
  app->SetAttribute("PathLossExp", DoubleValue(pathLossExp));
  app->SetAttribute("ShadowingSigmaDb", DoubleValue(shadowingSigmaDb));
  app->SetAttribute("NoiseFigureDb", DoubleValue(noiseFigureDb));

  app->SetAttribute("UsePerTable", BooleanValue(usePerTable));
  app->SetAttribute("PerTablePath", StringValue(perTablePath));
  app->SetAttribute("BwHz", UintegerValue(bwHz));
  app->SetAttribute("CrDen", UintegerValue(static_cast<uint8_t>(crDen)));
  app->SetAttribute("CfoLambda", DoubleValue(cfoLambda));
  app->SetAttribute("ValidationMode", BooleanValue(validationMode));

  app->SetAttribute("RxDutyCycleEnabled", BooleanValue(rxDutyCycleEnabled));
  app->SetAttribute("RxWindowMs", DoubleValue(rxWindowMs));
  app->SetAttribute("RxIdleMs", DoubleValue(rxIdleMs));
  app->SetAttribute("AckBatchSize", UintegerValue(ackBatchSize));

  // ---------------------------------------------------------------------------
  // CSV output
  // ---------------------------------------------------------------------------
  std::ofstream out(outCsv, std::ios::out | std::ios::trunc);
  if (!out.is_open())
    {
      std::cerr << "Cannot open outCsv=" << outCsv << "\n";
      return 1;
    }

  WriteHeader(out);

  // ---------------------------------------------------------------------------
  // Main exchange loop
  // ---------------------------------------------------------------------------
  NS_ABORT_MSG_IF(periodMs == 0, "periodMs must be > 0");

  const uint64_t nExchanges =
      static_cast<uint64_t>(
          std::floor(simTimeS * 1000.0 / static_cast<double>(periodMs)));

  double nowS = 0.0;

  for (uint64_t i = 0; i < nExchanges; ++i)
    {
      const auto r = app->RunOneExchange(nowS, logicalChannels);

      out << environment << ","
          << sf << ","
          << variant << ","
          << (validationMode ? 1 : 0) << ","

          // Link / PHY
          << r.snrDb << ","
          << r.rxDbm << ","
          << r.marginDb << ","
          << r.noiseDbm << ","
          << r.pathLossDb << ","

          // Payload configuration
          << r.payloadBytesConfig << ","
          << r.payloadBytesEffective << ","
          << r.g2gOverheadBytes << ","
          << r.phyPayloadBytesFragAvg << ","

          // Fragmentation
          << r.pkFragments << ","
          << r.ctFragments << ","
          << r.fragGapMs << ","
          << r.nFragmentsExchange << ","

          // Fragment outcomes
          << r.okFrags << ","
          << r.failFrags << ","
          << r.perObserved << ","

          // Airtime
          << r.txAirtimeMsTotal << ","
          << r.airtimeMsFragAvg << ","
          << r.exchangeLatencyS << ","

          // Energy
          << r.energyJExchange << ","
          << r.energyJExchangeTx << ","
          << r.energyJExchangeRx << ","
          << r.airtimeEnergyTx << ","
          << r.airtimeEnergyRx << ","
          << r.energyPerFragmentTx << ","
          << r.energyPerFragmentRx << ","

          // Interference / collision model
          << r.lambdaEff << ","
          << r.isOn << ","
          << r.attemptsTotal << ","
          << r.attemptsOn << ","
          << r.attemptsOff << ","
          << r.attemptsOffFraction << ","
          << r.lambdaEffAttemptAvg << ","

          // Error model
          << r.perFragmentAvg << ","
          << r.perCollisionSimAvg << ","
          << r.perCollisionThAvg << ","
          << r.perChannelAvg << ","
          << r.perTotalAvg << ","

          // Protocol configuration
          << app->GetPacing() << ","
          << r.sfAwarePacingThreshold << ","
          << r.retryMarginDb << ","
          << (r.rxDutyCycleEnabled ? 1 : 0) << ","
          << r.rxWindowMs << ","
          << r.rxIdleMs << ","
          << r.ackBatchSize << ","

          // Frame counters
          << r.dataFramesTx << ","
          << r.ackFramesTx << ","
          << r.framesTxTotal << ","
          << r.ackAirtimeMs << ","

          // ACK behaviour
          << r.ackLogicalFailures << ","
          << r.lostAckSleep << ","
          << r.lostAckChannel << ","
          << r.ackBatchRetxRounds << ","
          << r.ackPerChannelAvg << ","
          << r.ackPerCollisionAvg << ","
          << r.ackPerTotalAvg << ","

          // Receiver sleep losses
          << r.lostRxSleepData << ","
          << r.lostRxSleepAck << ","

          // Exchange outcome
          << (r.perExchangeFail ? 1 : 0) << ","
          << r.retransmissionsCount
          << "\n";

      if (timeBase == "end_to_start")
        {
          nowS += r.exchangeLatencyS;
          nowS += std::max(0.0, idleAfterExchangeMs) / 1000.0;
        }
      else
        {
          nowS += static_cast<double>(periodMs) / 1000.0;
        }

      NS_ASSERT(r.framesTxTotal == r.dataFramesTx + r.ackFramesTx);
    }

  out.close();
  return 0;
}
