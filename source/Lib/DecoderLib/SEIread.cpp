/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 \file     SEIread.cpp
 \brief    reading funtionality for SEI messages
 */

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Slice.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"
#include <iomanip>


//! \ingroup DecoderLib
//! \{

void SEIReader::sei_read_scode(std::ostream *pOS, uint32_t length, int& code, const char *pSymbolName)
{
  xReadSCode(length, code, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << code << "\n";
  }
}

void SEIReader::sei_read_code(std::ostream *pOS, uint32_t length, uint32_t &ruiCode, const char *pSymbolName)
{
  xReadCode(length, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_uvlc(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  xReadUvlc(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_svlc(std::ostream *pOS, int& ruiCode, const char *pSymbolName)
{
  xReadSvlc(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_flag(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  xReadFlag(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << (ruiCode?1:0) << "\n";
  }
}

void SEIReader::sei_read_string(std::ostream* os, std::string& code, const char* symbolName)
{
  xReadString(code, symbolName);
  if (os)
  {
    (*os) << "  " << std::setw(55) << symbolName << ": " << code << "\n";
  }
}


static inline void output_sei_message_header(SEI &sei, std::ostream *pDecodedMessageOutputStream, uint32_t payloadSize)
{
  if (pDecodedMessageOutputStream)
  {
    std::string seiMessageHdr(SEI::getSEIMessageString(sei.payloadType())); seiMessageHdr+=" SEI message";
    (*pDecodedMessageOutputStream) << std::setfill('-') << std::setw((int) seiMessageHdr.size()) << "-"
                                   << std::setfill(' ') << "\n"
                                   << seiMessageHdr << " (" << payloadSize << " bytes)"
                                   << "\n";
  }
}

/**
 * unmarshal a single SEI message from bitstream bs
 */
 // note: for independent parsing no parameter set should not be required here
SEIMessages::iterator SEIReader::parseSEImessage(InputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream)
{
  SEIMessages   seiListInCurNalu;
  setBitstream(bs);
  CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");

  SEIMessages::iterator newSEI = seis.end();

  do
  {
    const bool seiMessageRead = xReadSEImessage(seis, nalUnitType, nuh_layer_id, temporalId, vps, sps, hrd, pDecodedMessageOutputStream);
    if (seiMessageRead)
    {
      seiListInCurNalu.push_back(seis.back());
      if (newSEI == seis.end())
      {
        newSEI = --seis.end();
      }
    }
    /* SEI messages are an integer number of bytes, something has failed
    * in the parsing if bitstream not byte-aligned */
    CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");
  }
  while (m_pcBitstream->getNumBitsLeft() > 8);

  const SEIMessages fillerData = getSeisByType(seiListInCurNalu, SEI::PayloadType::FILLER_PAYLOAD);
  CHECK(fillerData.size() > 0 && fillerData.size() != seiListInCurNalu.size(), "When an SEI NAL unit contains an SEI message with payloadType equal to filler payload, the SEI NAL unit shall not contain any other SEI message with payloadType not equal to filler payload");
  const SEIMessages pictureTiming = getSeisByType(seiListInCurNalu, SEI::PayloadType::PICTURE_TIMING);
  CHECK(hrd.getGeneralHrdParameters().getGeneralSamePicTimingInAllOlsFlag() && pictureTiming.size() > 0 && pictureTiming.size() != seiListInCurNalu.size(),
        "When general_same_pic_timing_in_all_ols_flag is equal to 1 [...], and when an SEI NAL unit contains a non-scalable-nested SEI message with "
        "payloadType equal to 1 (PT), the SEI NAL unit shall not contain any other SEI message with payloadType not equal 1.");

  xReadRbspTrailingBits();

  return newSEI;
}

void SEIReader::parseAndExtractSEIScalableNesting(InputBitstream *bs, const NalUnitType nalUnitType,
                                                  const uint32_t nuh_layer_id, const VPS *vps, const SPS *sps, HRD &hrd,
                                                  uint32_t payloadSize, std::vector<SeiPayload> *seiList)
{
  SEIScalableNesting sn;
  setBitstream(bs);
  xParseSEIScalableNesting(sn, nalUnitType, nuh_layer_id, payloadSize, vps, sps, m_nestedHrd, nullptr, seiList);
  int payloadBitsRemaining = getBitstream()->getNumBitsLeft();
  if (payloadBitsRemaining) /* more_data_in_payload() */
  {
    for (; payloadBitsRemaining > 9; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_code(nullptr, 1, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    /* 2 */
    int finalBits = getBitstream()->peekBits(payloadBitsRemaining);
    int finalPayloadBits = 0;
    for (int mask = 0xff; finalBits & (mask >> finalPayloadBits); finalPayloadBits++)
    {
      continue;
    }

    /* 3 */
    for (; payloadBitsRemaining > 9 - finalPayloadBits; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_flag ( 0, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    uint32_t dummy;
    sei_read_flag( 0, dummy, "payload_bit_equal_to_one"); payloadBitsRemaining--;
    while (payloadBitsRemaining)
    {
      sei_read_flag( 0, dummy, "payload_bit_equal_to_zero"); payloadBitsRemaining--;
    }
  }
}

void SEIReader::getSEIDecodingUnitInfoDuiIdx(InputBitstream* bs, const uint32_t nuhLayerId, HRD& hrd,
                                             uint32_t payloadSize, int& duiIdx)
{
  const SEIBufferingPeriod* bp = hrd.getBufferingPeriodSEI();
  if (bp != nullptr)
  {
    InputBitstream  bsTmp(*bs);
    setBitstream(&bsTmp);

    SEIDecodingUnitInfo dui;
    xParseSEIDecodingUnitInfo(dui, payloadSize, *bp, nuhLayerId, nullptr);
    duiIdx = dui.decodingUnitIdx;

    setBitstream(bs);
  }
}

bool SEIReader::xCheckNnpfcSeiMsg(uint32_t seiId, bool baseFlag, const std::vector<int> nnpfcValueList)
{
  if (baseFlag)
  {
    //Check if this is a new filter or a repetition of an existing base flag
    for (auto val : nnpfcValueList)
    {
      if (val == seiId)
      {
        //The filter is a repetition.
        return false;
      }
    }
  }
  else
  {
    bool filterHasPresent = false;
    for(auto val : nnpfcValueList)
    {
      if (val == seiId)
      {
        filterHasPresent = true;
        break;
      }
    }
    CHECK(!filterHasPresent, "Cannot have update filter without base filter already present!")
  }
  return true;
}

bool SEIReader::xCheckNnpfcUpdatePresentSeiMsg(uint32_t seiId, const std::vector<int> nnpfcValueList)
{
  int count = 0;
  for (auto val : nnpfcValueList)
  {
    if (val == seiId)
    {
      count++;
      if (count == 2)
      {
        return true;
      }
    }
  }
  return false;
}

bool SEIReader::xReadSEImessage(SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream)
{
#if ENABLE_TRACING
  xTraceSEIHeader();
#endif
  int payloadType = 0;
  uint32_t val = 0;

  do
  {
    sei_read_code(nullptr, 8, val, "payload_type");
    payloadType += val;
  } while (val==0xFF);

  uint32_t payloadSize = 0;
  do
  {
    sei_read_code(nullptr, 8, val, "payload_size");
    payloadSize += val;
  } while (val==0xFF);

#if ENABLE_TRACING
  xTraceSEIMessageType((SEI::PayloadType)payloadType);
#endif

  /* extract the payload for this single SEI message.
   * This allows greater safety in erroneous parsing of an SEI message
   * from affecting subsequent messages.
   * After parsing the payload, bs needs to be restored as the primary
   * bitstream.
   */
  InputBitstream *bs = getBitstream();
  setBitstream(bs->extractSubstream(payloadSize * 8));

  SEI* sei = nullptr;

  if(nalUnitType == NAL_UNIT_PREFIX_SEI)
  {
    switch (SEI::PayloadType(payloadType))
    {
    case SEI::PayloadType::FILLER_PAYLOAD:
      sei = new SEIFillerPayload;
      xParseSEIFillerPayload((SEIFillerPayload&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::USER_DATA_UNREGISTERED:
      {
        auto udu = new SEIUserDataUnregistered;
        xParseSEIuserDataUnregistered(*udu, payloadSize, pDecodedMessageOutputStream);
        sei = udu;
        break;
      }
    case SEI::PayloadType::DECODING_UNIT_INFO:
      {
        const SEIBufferingPeriod* bp = hrd.getBufferingPeriodSEI();
        if (bp == nullptr)
        {
          msg(WARNING, "Warning: Found Decoding unit information SEI message, but no active buffering period is "
                       "available. Ignoring.");
        }
        else
        {
          sei = new SEIDecodingUnitInfo;
          xParseSEIDecodingUnitInfo((SEIDecodingUnitInfo&) *sei, payloadSize, *bp, temporalId,
                                    pDecodedMessageOutputStream);
        }
        break;
      }
    case SEI::PayloadType::BUFFERING_PERIOD:
      {
        auto bp = new SEIBufferingPeriod;
        xParseSEIBufferingPeriod(*bp, payloadSize, pDecodedMessageOutputStream);
        hrd.setBufferingPeriodSEI(bp);
        sei = bp;
        break;
      }
    case SEI::PayloadType::PICTURE_TIMING:
      {
        const SEIBufferingPeriod* bp = hrd.getBufferingPeriodSEI();
        if (bp == nullptr)
        {
          msg(WARNING,
              "Warning: Found Picture timing SEI message, but no active buffering period is available. Ignoring.");
        }
        else
        {
          sei = new SEIPictureTiming;
          xParseSEIPictureTiming((SEIPictureTiming&) *sei, payloadSize, temporalId, *bp, pDecodedMessageOutputStream);
          hrd.setPictureTimingSEI((SEIPictureTiming*) sei);
        }
        break;
      }
    case SEI::PayloadType::SCALABLE_NESTING:
      {
        auto sn = new SEIScalableNesting;
        xParseSEIScalableNesting(*sn, nalUnitType, nuh_layer_id, payloadSize, vps, sps, hrd,
                                 pDecodedMessageOutputStream, nullptr);
        sei = sn;
        break;
      }
    case SEI::PayloadType::FRAME_FIELD_INFO:
      sei = new SEIFrameFieldInfo;
      xParseSEIFrameFieldinfo((SEIFrameFieldInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::DEPENDENT_RAP_INDICATION:
      sei = new SEIDependentRAPIndication;
      xParseSEIDependentRAPIndication((SEIDependentRAPIndication &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::EXTENDED_DRAP_INDICATION:
      sei = new SEIExtendedDrapIndication;
      xParseSEIExtendedDrapIndication((SEIExtendedDrapIndication &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::FRAME_PACKING:
      sei = new SEIFramePacking;
      xParseSEIFramePacking((SEIFramePacking &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::DISPLAY_ORIENTATION:
      sei = new SEIDisplayOrientation;
      xParseSEIDisplayOrientation((SEIDisplayOrientation &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::ANNOTATED_REGIONS:
      sei = new SEIAnnotatedRegions;
      xParseSEIAnnotatedRegions((SEIAnnotatedRegions &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::PARAMETER_SETS_INCLUSION_INDICATION:
      sei = new SEIParameterSetsInclusionIndication;
      xParseSEIParameterSetsInclusionIndication((SEIParameterSetsInclusionIndication &) *sei, payloadSize,
                                                pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::MASTERING_DISPLAY_COLOUR_VOLUME:
      sei = new SEIMasteringDisplayColourVolume;
      xParseSEIMasteringDisplayColourVolume((SEIMasteringDisplayColourVolume &) *sei, payloadSize,
                                            pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
      sei = new SEIAlternativeTransferCharacteristics;
      xParseSEIAlternativeTransferCharacteristics((SEIAlternativeTransferCharacteristics &) *sei, payloadSize,
                                                  pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::EQUIRECTANGULAR_PROJECTION:
      sei = new SEIEquirectangularProjection;
      xParseSEIEquirectangularProjection((SEIEquirectangularProjection &) *sei, payloadSize,
                                         pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SPHERE_ROTATION:
      sei = new SEISphereRotation;
      xParseSEISphereRotation((SEISphereRotation &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::OMNI_VIEWPORT:
      sei = new SEIOmniViewport;
      xParseSEIOmniViewport((SEIOmniViewport &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::REGION_WISE_PACKING:
      sei = new SEIRegionWisePacking;
      xParseSEIRegionWisePacking((SEIRegionWisePacking &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::GENERALIZED_CUBEMAP_PROJECTION:
      sei = new SEIGeneralizedCubemapProjection;
      xParseSEIGeneralizedCubemapProjection((SEIGeneralizedCubemapProjection &) *sei, payloadSize,
                                            pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SCALABILITY_DIMENSION_INFO:
      sei = new SEIScalabilityDimensionInfo;
      xParseSEIScalabilityDimensionInfo((SEIScalabilityDimensionInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::GREEN_METADATA:
      sei = new SEIGreenMetadataInfo;
      xParseSEIGreenMetadataInfo((SEIGreenMetadataInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO:
      sei = new SEIMultiviewAcquisitionInfo;
      xParseSEIMultiviewAcquisitionInfo((SEIMultiviewAcquisitionInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::MULTIVIEW_VIEW_POSITION:
      sei = new SEIMultiviewViewPosition;
      xParseSEIMultiviewViewPosition((SEIMultiviewViewPosition &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::ALPHA_CHANNEL_INFO:
      sei = new SEIAlphaChannelInfo;
      xParseSEIAlphaChannelInfo((SEIAlphaChannelInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::DEPTH_REPRESENTATION_INFO:
      sei = new SEIDepthRepresentationInfo;
      xParseSEIDepthRepresentationInfo((SEIDepthRepresentationInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SUBPICTURE_LEVEL_INFO:
      sei = new SEISubpictureLevelInfo;
      xParseSEISubpictureLevelInfo((SEISubpictureLevelInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SAMPLE_ASPECT_RATIO_INFO:
      sei = new SEISampleAspectRatioInfo;
      xParseSEISampleAspectRatioInfo((SEISampleAspectRatioInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::USER_DATA_REGISTERED_ITU_T_T35:
      sei = new SEIUserDataRegistered;
      xParseSEIUserDataRegistered((SEIUserDataRegistered &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS:
      sei = new SEIFilmGrainCharacteristics;
      xParseSEIFilmGrainCharacteristics((SEIFilmGrainCharacteristics &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::CONTENT_LIGHT_LEVEL_INFO:
      sei = new SEIContentLightLevelInfo;
      xParseSEIContentLightLevelInfo((SEIContentLightLevelInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::AMBIENT_VIEWING_ENVIRONMENT:
      sei = new SEIAmbientViewingEnvironment;
      xParseSEIAmbientViewingEnvironment((SEIAmbientViewingEnvironment &) *sei, payloadSize,
                                         pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::CONTENT_COLOUR_VOLUME:
      sei = new SEIContentColourVolume;
      xParseSEIContentColourVolume((SEIContentColourVolume &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::COLOUR_TRANSFORM_INFO:
      sei = new SEIColourTransformInfo;
      xParseSEIColourTransformInfo((SEIColourTransformInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SEI_MANIFEST:
      sei = new SEIManifest;
      xParseSEISEIManifest((SEIManifest&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SEI_PREFIX_INDICATION:
      sei = new SEIPrefixIndication;
      xParseSEISEIPrefixIndication((SEIPrefixIndication&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::CONSTRAINED_RASL_ENCODING:
      sei = new SEIConstrainedRaslIndication;
      xParseSEIConstrainedRaslIndication((SEIConstrainedRaslIndication &) *sei, payloadSize,
                                         pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SHUTTER_INTERVAL_INFO:
      sei = new SEIShutterIntervalInfo;
      xParseSEIShutterInterval((SEIShutterIntervalInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS:
      sei = new SEINeuralNetworkPostFilterCharacteristics;
      xParseSEINNPostFilterCharacteristics((SEINeuralNetworkPostFilterCharacteristics &) *sei, payloadSize, sps,
                                           pDecodedMessageOutputStream);
        
      if (xCheckNnpfcSeiMsg( ((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id, ((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_baseFlag, nnpfcValues) )
      {
        nnpfcValues.push_back(((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id);
      }
      break;
    case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_ACTIVATION:
      sei = new SEINeuralNetworkPostFilterActivation;
      xParseSEINNPostFilterActivation((SEINeuralNetworkPostFilterActivation &) *sei, payloadSize,
                                      pDecodedMessageOutputStream);
      nnpfcProcessed = false;
      CHECK(nnpfcValues.size() == 0, "At leaset one NNPFC SEI message should precede NNPFA")
      for(int i=0; i<nnpfcValues.size(); ++i)
      {
        if(((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id == nnpfcValues[i])
        {
          //In the case that the NNPFA activates a non-base filter, only consider it process when we have NNPFC that updates the base filter present
          if(((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_baseFlag ||
            (!((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_baseFlag && xCheckNnpfcUpdatePresentSeiMsg( ((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id, nnpfcValues)) )
          {
            nnpfcProcessed = true;
          }
        }
      }
      CHECK(!nnpfcProcessed, "No NNPFC, no NNPFA")
      nnpfcProcessed = false;
      break;
    case SEI::PayloadType::PHASE_INDICATION:
      sei = new SEIPhaseIndication;
      xParseSEIPhaseIndication((SEIPhaseIndication &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    default:
      for (uint32_t i = 0; i < payloadSize; i++)
      {
        uint32_t seiByte;
        sei_read_code(nullptr, 8, seiByte, "unknown prefix SEI payload byte");
      }
      msg(WARNING, "Unknown prefix SEI message (payloadType = %d) was found!\n", payloadType);
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "Unknown prefix SEI message (payloadType = " << payloadType
                                       << ") was found!\n";
      }
      break;
    }
  }
  else
  {
    switch (SEI::PayloadType(payloadType))
    {
    case SEI::PayloadType::USER_DATA_UNREGISTERED:
      {
        auto udu = new SEIUserDataUnregistered;
        xParseSEIuserDataUnregistered(*udu, payloadSize, pDecodedMessageOutputStream);
        sei = udu;
        break;
      }
    case SEI::PayloadType::DECODED_PICTURE_HASH:
      sei = new SEIDecodedPictureHash;
      xParseSEIDecodedPictureHash((SEIDecodedPictureHash &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::SCALABLE_NESTING:
      {
        auto sn = new SEIScalableNesting;
        xParseSEIScalableNesting(*sn, nalUnitType, nuh_layer_id, payloadSize, vps, sps, hrd,
                                 pDecodedMessageOutputStream, nullptr);
        sei = sn;
        break;
      }
    case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS:
      sei = new SEINeuralNetworkPostFilterCharacteristics;
      xParseSEINNPostFilterCharacteristics((SEINeuralNetworkPostFilterCharacteristics &) *sei, payloadSize, sps,
        pDecodedMessageOutputStream);

      if (xCheckNnpfcSeiMsg( ((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id, ((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_baseFlag, nnpfcValues) )
      {
        nnpfcValues.push_back(((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id);
      }
      break;
    case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_ACTIVATION:
      sei = new SEINeuralNetworkPostFilterActivation;
      xParseSEINNPostFilterActivation((SEINeuralNetworkPostFilterActivation &) *sei, payloadSize,
        pDecodedMessageOutputStream);
      nnpfcProcessed = false;
      CHECK(nnpfcValues.size() == 0, "At leaset one NNPFC SEI message should precede NNPFA")
      for (int i = 0; i < nnpfcValues.size(); ++i)
      {
        if (((SEINeuralNetworkPostFilterCharacteristics*) sei)->m_id == nnpfcValues[i])
        {
          nnpfcProcessed = true;
        }
      }
      CHECK(!nnpfcProcessed, "No NNPFC, no NNPFA")
      nnpfcProcessed = false;
      break;
    case SEI::PayloadType::FILLER_PAYLOAD:
      sei = new SEIFillerPayload;
      xParseSEIFillerPayload((SEIFillerPayload &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    default:
      for (uint32_t i = 0; i < payloadSize; i++)
      {
        uint32_t seiByte;
        sei_read_code(nullptr, 8, seiByte, "unknown suffix SEI payload byte");
      }
      msg(WARNING, "Unknown suffix SEI message (payloadType = %d) was found!\n", payloadType);
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "Unknown suffix SEI message (payloadType = " << payloadType
                                       << ") was found!\n";
      }
      break;
    }
  }

  if (sei != nullptr)
  {
    seis.push_back(sei);
  }

  /* By definition the underlying bitstream terminates in a byte-aligned manner.
   * 1. Extract all bar the last MIN(bitsremaining,nine) bits as reserved_payload_extension_data
   * 2. Examine the final 8 bits to determine the payload_bit_equal_to_one marker
   * 3. Extract the remainingreserved_payload_extension_data bits.
   *
   * If there are fewer than 9 bits available, extract them.
   */
  int payloadBitsRemaining = getBitstream()->getNumBitsLeft();
  if (payloadBitsRemaining) /* more_data_in_payload() */
  {
    for (; payloadBitsRemaining > 9; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_code ( pDecodedMessageOutputStream, 1, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    /* 2 */
    int finalBits = getBitstream()->peekBits(payloadBitsRemaining);
    int finalPayloadBits = 0;
    for (int mask = 0xff; finalBits & (mask >> finalPayloadBits); finalPayloadBits++)
    {
      continue;
    }

    /* 3 */
    for (; payloadBitsRemaining > 9 - finalPayloadBits; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_flag ( 0, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    uint32_t dummy;
    sei_read_flag( 0, dummy, "payload_bit_equal_to_one"); payloadBitsRemaining--;
    while (payloadBitsRemaining)
    {
      sei_read_flag( 0, dummy, "payload_bit_equal_to_zero"); payloadBitsRemaining--;
    }
  }

  /* restore primary bitstream for sei_message */
  delete getBitstream();
  setBitstream(bs);

  return sei != nullptr;
}

void SEIReader::xParseSEIFillerPayload(SEIFillerPayload &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  for (uint32_t i = 0; i < payloadSize; i++)
  {
    uint32_t val;
    sei_read_code( nullptr, 8, val, "ff_byte");
    CHECK(val != 0xff, "ff_byte shall be a byte having the value 0xFF");
  }
}

/**
 * parse bitstream bs and unpack a user_data_unregistered SEI message
 * of payloasSize bytes into sei.
 */

void SEIReader::xParseSEIuserDataUnregistered(SEIUserDataUnregistered& sei, uint32_t payloadSize,
                                              std::ostream* pDecodedMessageOutputStream)
{
  CHECK(payloadSize < sei.uuid.size(), "Payload too small");
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  for (uint32_t i = 0; i < sei.uuid.size(); i++)
  {
    sei_read_code(pDecodedMessageOutputStream, 8, val, "uuid_iso_iec_11578");
    sei.uuid[i] = val;
  }

  sei.data.resize(payloadSize - sei.uuid.size());

  for (uint32_t i = 0; i < sei.data.size(); i++)
  {
    sei_read_code(nullptr, 8, val, "user_data_payload_byte");
    sei.data[i] = val;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  User data payload size: " << sei.data.size() << "\n";
  }
}

void SEIReader::xParseSEIShutterInterval(SEIShutterIntervalInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int32_t i;
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code(pDecodedMessageOutputStream, 32, val, "sii_time_scale");                      sei.m_siiTimeScale = val;
  sei_read_flag(pDecodedMessageOutputStream, val, "fixed_shutter_interval_within_clvs_flag"); sei.m_siiFixedSIwithinCLVS = val;
  if (sei.m_siiFixedSIwithinCLVS)
  {
    sei_read_code(pDecodedMessageOutputStream, 32, val, "sii_num_units_in_shutter_interval");   sei.m_siiNumUnitsInShutterInterval = val;
  }
  else
  {
    sei_read_code(pDecodedMessageOutputStream, 3, val, "sii_max_sub_layers_minus1 ");          sei.m_siiMaxSubLayersMinus1 = val;
    sei.m_siiSubLayerNumUnitsInSI.resize(sei.m_siiMaxSubLayersMinus1 + 1);
    for (i = 0; i <= sei.m_siiMaxSubLayersMinus1; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "sub_layer_num_units_in_shutter_interval[ i ]");
      sei.m_siiSubLayerNumUnitsInSI[i] = val;
    }
  }
}


/**
 * parse bitstream bs and unpack a decoded picture hash SEI message
 * of payloadSize bytes into sei.
 */
void SEIReader::xParseSEIDecodedPictureHash(SEIDecodedPictureHash& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t bytesRead = 0;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream, 8, val, "dph_sei_hash_type");
  sei.method = static_cast<HashType>(val); bytesRead++;
  sei_read_code( pDecodedMessageOutputStream, 1, val, "dph_sei_single_component_flag");
  sei.singleCompFlag = val;
  sei_read_code( pDecodedMessageOutputStream, 7, val, "dph_sei_reserved_zero_7bits");
  bytesRead++;
  uint32_t expectedSize =
    (sei.singleCompFlag ? 1 : 3) * (sei.method == HashType::MD5 ? 16 : (sei.method == HashType::CRC ? 2 : 4));
  CHECK ((payloadSize - bytesRead) != expectedSize, "The size of the decoded picture hash does not match the expected size.");

  const char *traceString="\0";
  switch (sei.method)
  {
  case HashType::MD5:
    traceString = "picture_md5";
    break;
  case HashType::CRC:
    traceString = "picture_crc";
    break;
  case HashType::CHECKSUM:
    traceString = "picture_checksum";
    break;
  default:
    THROW("Unknown hash type");
    break;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  " << std::setw(55) << traceString << ": " << std::hex << std::setfill('0');
  }

  sei.m_pictureHash.hash.clear();
  for(;bytesRead < payloadSize; bytesRead++)
  {
    sei_read_code(nullptr, 8, val, traceString);
    sei.m_pictureHash.hash.push_back((uint8_t)val);
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << std::setw(2) << val;
    }
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << std::dec << std::setfill(' ') << "\n";
  }
}

void SEIReader::xParseSEIScalableNesting(SEIScalableNesting& sn, const NalUnitType nalUnitType,
                                         const uint32_t nuhLayerId, uint32_t payloadSize, const VPS* vps,
                                         const SPS* sps, HRD& hrd, std::ostream* decodedMessageOutputStream,
                                         std::vector<SeiPayload>* seiList)
{
  uint32_t symbol;
  output_sei_message_header(sn, decodedMessageOutputStream, payloadSize);

  sei_read_flag(decodedMessageOutputStream, symbol, "sn_ols_flag");
  const bool hasOldIdx = symbol != 0;

  sei_read_flag(decodedMessageOutputStream, symbol, "sn_subpic_flag");
  const bool hasSubpicId = symbol != 0;

  if (hasOldIdx)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_olss_minus1");
    sn.olsIdx.resize(symbol + 1);

    for (uint32_t i = 0; i < sn.olsIdx.size(); i++)
    {
      const uint32_t pred = i == 0 ? 0 : sn.olsIdx[i - 1] + 1;

      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_ols_idx_delta_minus1[i]");
      sn.olsIdx[i] = pred + symbol;
    }
    if (vps && vps->getVPSId() != 0)
    {
      uint32_t lowestLayerId = std::numeric_limits<uint32_t>::max();
      for (uint32_t olsIdx: sn.olsIdx)
      {
        for (int layerIdx = 0; layerIdx < vps->getNumLayersInOls(olsIdx); layerIdx++)
        {
          lowestLayerId = std::min(lowestLayerId, vps->getLayerIdInOls(olsIdx, layerIdx));
        }
      }
      CHECK(lowestLayerId != nuhLayerId,
            "nuh_layer_id is not equal to the lowest layer among Olss that the scalable SEI applies");
    }
  }
  else
  {
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_all_layers_flag");
    const bool allLayersFlag = symbol != 0;
    if (!allLayersFlag)
    {
      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_layers_minus1");
      sn.layerId.resize(symbol + 1);
      sn.layerId[0] = nuhLayerId;
      for (uint32_t i = 1; i < sn.layerId.size(); i++)
      {
        sei_read_code(decodedMessageOutputStream, 6, symbol, "sn_layer_id[i]");
        sn.layerId[i] = symbol;
      }
    }
    else
    {
      sn.layerId.clear();
    }
  }
  if (hasSubpicId)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_subpics_minus1");
    sn.subpicId.resize(symbol + 1);

    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_subpic_id_len_minus1");
    sn.subpicIdLen = symbol + 1;

    for (uint32_t i = 0; i < sn.subpicId.size(); i++)
    {
      sei_read_code(decodedMessageOutputStream, sn.subpicIdLen, symbol, "sn_subpic_id[i]");
      sn.subpicId[i] = symbol;
    }
  }

  sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_seis_minus1");
  const uint32_t numSeis = symbol + 1;
  CHECK(numSeis > 64, "The value of sn_num_seis_minus1 shall be in the range of 0 to 63");

  // byte alignment
  while (m_pcBitstream->getNumBitsRead() % 8 != 0)
  {
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_zero_bit");
  }

  if (seiList == nullptr)
  {
    // read nested SEI messages
    for (int i = 0; i < numSeis; i++)
    {
      SEIMessages tmpSEIs;
      const bool  seiMessageRead =
        xReadSEImessage(tmpSEIs, nalUnitType, nuhLayerId, 0, vps, sps, m_nestedHrd, decodedMessageOutputStream);
      if (seiMessageRead)
      {
        if (tmpSEIs.front()->payloadType() == SEI::PayloadType::BUFFERING_PERIOD)
        {
          auto bp = reinterpret_cast<SEIBufferingPeriod*>(tmpSEIs.front());
          m_nestedHrd.setBufferingPeriodSEI(bp);
          const SEIBufferingPeriod* nonNestedBp = hrd.getBufferingPeriodSEI();
          if (nonNestedBp)
          {
            checkBPSyntaxElementLength(nonNestedBp, bp);
          }
        }
        sn.nestedSeis.push_back(tmpSEIs.front());
        tmpSEIs.clear();
      }
    }

    const GeneralHrdParams* generalHrd = vps && vps->getVPSGeneralHrdParamsPresentFlag()
                                           ? vps->getGeneralHrdParameters()
                                         : sps->getGeneralHrdParametersPresentFlag() ? sps->getGeneralHrdParameters()
                                                                                     : nullptr;

    xCheckScalableNestingConstraints(sn, nalUnitType, generalHrd);
  }
  else
  {
    for (int i = 0; i < numSeis; i++)
    {
      uint32_t payloadTypeVal = 0;
      do
      {
        sei_read_code(nullptr, 8, symbol, "payload_type");
        payloadTypeVal += symbol;
      } while (symbol == 0xff);

      auto payloadType = static_cast<SEI::PayloadType>(payloadTypeVal);

      uint32_t payloadSize = 0;
      do
      {
        sei_read_code(nullptr, 8, symbol, "payload_size");
        payloadSize += symbol;
      } while (symbol == 0xff);

      int duiIdx = 0;
      if (SEI::PayloadType(payloadType) == SEI::PayloadType::DECODING_UNIT_INFO)
      {
        getSEIDecodingUnitInfoDuiIdx(getBitstream(), nuhLayerId, hrd, payloadSize, duiIdx);
      }

      auto payload = new uint8_t[payloadSize];
      for (uint32_t j = 0; j < payloadSize; j++)
      {
        sei_read_code(nullptr, 8, symbol, "payload_content");
        payload[j] = symbol;
      }

      auto&&   subpicId    = !sn.subpicId.empty() ? sn.subpicId : std::vector<uint16_t>{ 0 };
      uint8_t* payloadTemp = payload;

      if (!sn.olsIdx.empty())
      {
        for (uint32_t j = 0; j < sn.olsIdx.size(); j++)
        {
          for (uint32_t k = 0; k < subpicId.size(); k++)
          {
            if (j != 0 || k != 0)
            {
              payloadTemp = new uint8_t[payloadSize];
              std::copy_n(payload, payloadSize, payloadTemp);
            }

            seiList->push_back(
              SeiPayload{ payloadType, sn.olsIdx[j], false, payloadSize, payloadTemp, duiIdx, subpicId[k] });
          }
        }
      }
      else if (sn.allLayersFlag())
      {
        for (uint32_t k = 0; k < subpicId.size(); k++)
        {
          if (k != 0)
          {
            payloadTemp = new uint8_t[payloadSize];
            std::copy_n(payload, payloadSize, payloadTemp);
          }

          seiList->push_back(
            SeiPayload{ payloadType, nuhLayerId, true, payloadSize, payloadTemp, duiIdx, subpicId[k] });
        }
      }
      else
      {
        for (uint32_t j = 0; j < sn.layerId.size(); j++)
        {
          for (uint32_t k = 0; k < subpicId.size(); k++)
          {
            if (j != 0 || k != 0)
            {
              payloadTemp = new uint8_t[payloadSize];
              std::copy_n(payload, payloadSize, payloadTemp);
            }

            seiList->push_back(
              SeiPayload{ payloadType, sn.layerId[j], false, payloadSize, payloadTemp, duiIdx, subpicId[k] });
          }
        }
      }
    }
  }

  if (decodedMessageOutputStream)
  {
    (*decodedMessageOutputStream) << "End of scalable nesting SEI message\n";
  }
}


void SEIReader::xParseSEIGreenMetadataInfo(SEIGreenMetadataInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code(pDecodedMessageOutputStream, 8, code, "green_metadata_type");
  sei.m_greenMetadataType = code;
  printf("GREEN MPEG Output: \n");
  printf("Metadata Type: %i\n", sei.m_greenMetadataType);

  switch (sei.m_greenMetadataType)
  {
  case 0:
    sei_read_code(pDecodedMessageOutputStream, 4, code, "period_type");
    sei.m_periodType = code;
    sei_read_code(pDecodedMessageOutputStream, 3, code, "granularity_type");
    sei.m_greenMetadataGranularityType = code;
    sei_read_code(pDecodedMessageOutputStream, 1, code, "extended_representation_flag");
    sei.m_greenMetadataExtendedRepresentation = code;

    printf ("Period Type: %i\n",sei.m_periodType);
    printf ("Granularity Type: %i\n",sei.m_greenMetadataGranularityType);
    printf ("Extended Representation Flag Type: %i\n",sei.m_greenMetadataExtendedRepresentation);

    if (sei.m_periodType == 2)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, code, "num_seconds");
      sei.m_numSeconds = code;
      printf ("Number of Seconds: %i\n",sei.m_numSeconds);
    }
    else if (sei.m_periodType == 3)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, code, "num_pictures");
      sei.m_numPictures = code;
      printf ("Number of Pictures: %i\n",sei.m_numPictures);
    }

    if (sei.m_greenMetadataGranularityType == 0)
    {
      sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_non_zero_blocks_area");
      sei.m_greenComplexityMetrics.portionNonZeroBlocksArea = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_non_zero_transform_coefficients_area");
      sei.m_greenComplexityMetrics.portionNonZeroTransformCoefficientsArea = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_intra_predicted_blocks_area");
      sei.m_greenComplexityMetrics.portionIntraPredictedBlocksArea = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_deblocking_instances");
      sei.m_greenComplexityMetrics.portionDeblockingInstances = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_alf_instances");
      sei.m_greenComplexityMetrics.portionAlfInstances = code;

      printf ("Portion Non Zero Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionNonZeroBlocksArea);
      printf ("Portion Non Zero Transform Coefficients Area: %i\n",sei.m_greenComplexityMetrics.portionNonZeroTransformCoefficientsArea);
      printf ("Portion Intra Predicted Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionIntraPredictedBlocksArea);
      printf ("Portion Deblocking Instances: %i\n",sei.m_greenComplexityMetrics.portionDeblockingInstances);
      printf ("Portion ALF Instances: %i\n",sei.m_greenComplexityMetrics.portionAlfInstances);

      if(sei.m_greenMetadataExtendedRepresentation == 1)
      {
        if(sei.m_greenComplexityMetrics.portionNonZeroBlocksArea != 0)
        {
          sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_non_zero_4_8_16_blocks_area");
          sei.m_greenComplexityMetrics.portionNonZero_4_8_16BlocksArea = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_non_zero_32_64_128_blocks_area");
          sei.m_greenComplexityMetrics.portionNonZero_32_64_128BlocksArea = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_non_zero_256_512_1024_blocks_area");
          sei.m_greenComplexityMetrics.portionNonZero_256_512_1024BlocksArea = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_non_zero_2048_4096_blocks_area");
          sei.m_greenComplexityMetrics.portionNonZero_2048_4096BlocksArea = code;
          printf ("Portion Non Zero 4/8/16 Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionNonZero_4_8_16BlocksArea);
          printf ("Portion Non Zero 32/64/128 Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionNonZero_32_64_128BlocksArea);
          printf ("Portion Non Zero 256/512/1024 Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionNonZero_256_512_1024BlocksArea);
          printf ("Portion Non Zero 2048/4096 Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionNonZero_2048_4096BlocksArea);
        }

        if(sei.m_greenComplexityMetrics.portionIntraPredictedBlocksArea < 255)
        {
          sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_bi_and_gpm_predicted_blocks_area");
          sei.m_greenComplexityMetrics.portionBiAndGpmPredictedBlocksArea = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_bdof_blocks_area");
          sei.m_greenComplexityMetrics.portionBdofBlocksArea = code;
          printf ("Portion BI and GPM Predicted Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionBiAndGpmPredictedBlocksArea);
          printf ("Portion BDOF Blocks Area: %i\n",sei.m_greenComplexityMetrics.portionBdofBlocksArea);
        }

        sei_read_code(pDecodedMessageOutputStream, 8, code, "portion_sao_instances");
        sei.m_greenComplexityMetrics.portionSaoInstances = code;
        printf ("Portion SAO Instances: %i\n",sei.m_greenComplexityMetrics.portionSaoInstances);
      }
    }
    break;
  case 1:
    sei_read_code(pDecodedMessageOutputStream, 16, code, "xsd_subpic_number_minus1");
    sei.m_xsdSubpicNumberMinus1 = code;
    printf("XSD Subpic Number of Metrics: %i\n", sei.m_xsdSubpicNumberMinus1 + 1);

    int xsdSubpicIdc;
    int xsdMetricNumberMinus1;

    for (int i = 0; i <= sei.m_xsdSubpicNumberMinus1; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, code, "xsd_subpic_idc[i]");
      xsdSubpicIdc = code;
      printf("XSD Subpic Idc[i]: %i\n", xsdSubpicIdc);

      sei_read_code(pDecodedMessageOutputStream, 8, code, "xsd_metric_number_minus1[i]");
      xsdMetricNumberMinus1 = code;
      printf("XSD Metric Number Minus1[i]: %i\n", xsdMetricNumberMinus1);

      int xsdMetricType;
      int xsdMetricValue;
      for (int j = 0; j <= xsdMetricNumberMinus1; j++)
      {
        sei_read_code(pDecodedMessageOutputStream, 8, code, "xsd_metric_type[i][j]");
        xsdMetricType = code;

        sei_read_code(pDecodedMessageOutputStream, 16, code, "xsd_metric_value[i][j]");
        xsdMetricValue = code;

        switch (xsdMetricType)
        {
        case 0: //PSNR
          sei.m_xsdMetricValuePSNR = code;
          printf("PSNR value: %0.2f\n", (double(xsdMetricValue)/100.0));
          break;
        case 1: //SSIM
          sei.m_xsdMetricValueSSIM = code;
          printf("SSIM value: %0.2f\n", double(xsdMetricValue/100.0));
          break;
        case 2:  //W-PSNR
          sei.m_xsdMetricValueWPSNR = code;
          printf("W-PSNR value: %0.2f\n", double(xsdMetricValue/100.0));
          break;
        case 3: //WS-PSNR
          sei.m_xsdMetricValueWSPSNR = code;
          printf("WS-PSNR value: %0.2f\n", double(xsdMetricValue/100.0));
          break;
        default: //User Defined
          break;
        }
      }
    }
    break;
  }
}

void SEIReader::xCheckScalableNestingConstraints(const SEIScalableNesting& sn, const NalUnitType nalUnitType,
                                                 const GeneralHrdParams* generalHrd)
{
  const std::vector<SEI::PayloadType> vclAssociatedSeiList{
    SEI::PayloadType::FILLER_PAYLOAD,
    SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS,
    SEI::PayloadType::FRAME_PACKING,
    SEI::PayloadType::GREEN_METADATA,
    SEI::PayloadType::PARAMETER_SETS_INCLUSION_INDICATION,
    SEI::PayloadType::MASTERING_DISPLAY_COLOUR_VOLUME,
    SEI::PayloadType::CONTENT_LIGHT_LEVEL_INFO,
    SEI::PayloadType::DEPENDENT_RAP_INDICATION,
    SEI::PayloadType::ALTERNATIVE_TRANSFER_CHARACTERISTICS,
    SEI::PayloadType::AMBIENT_VIEWING_ENVIRONMENT,
    SEI::PayloadType::CONTENT_COLOUR_VOLUME,
    SEI::PayloadType::EQUIRECTANGULAR_PROJECTION,
    SEI::PayloadType::GENERALIZED_CUBEMAP_PROJECTION,
    SEI::PayloadType::SPHERE_ROTATION,
    SEI::PayloadType::REGION_WISE_PACKING,
    SEI::PayloadType::OMNI_VIEWPORT,
    SEI::PayloadType::FRAME_FIELD_INFO,
    SEI::PayloadType::SAMPLE_ASPECT_RATIO_INFO,
  };

  bool containBPorPTorDUIorSLI = false;
  bool containNoBPorPTorDUIorSLI = false;

  for (auto nestedsei: sn.nestedSeis)
  {
    CHECK(nestedsei->payloadType() == SEI::PayloadType::FILLER_PAYLOAD
            || nestedsei->payloadType() == SEI::PayloadType::SCALABLE_NESTING,
          "An SEI message that has payloadType equal to filler payload or scalable nesting shall not be contained in a "
          "scalable nesting SEI message");

    CHECK(nestedsei->payloadType() == SEI::PayloadType::SCALABILITY_DIMENSION_INFO,
          "A scalability dimension information SEI message shall not be contained in a scalable nesting SEI message");
    CHECK(nestedsei->payloadType() == SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO,
          "A multiview acquisition information SEI message shall not be contained in a scalable nesting SEI message");

    CHECK(nestedsei->payloadType() != SEI::PayloadType::FILLER_PAYLOAD
            && nestedsei->payloadType() != SEI::PayloadType::DECODED_PICTURE_HASH && nalUnitType != NAL_UNIT_PREFIX_SEI,
          "When a scalable nesting SEI message contains an SEI message that has payloadType not equal to filler "
          "payload or decoded picture hash, the SEI NAL unit containing the scalable nesting SEI message shall have "
          "nal_unit_type equal to PREFIX_SEI_NUT");

    CHECK(
      nestedsei->payloadType() == SEI::PayloadType::DECODED_PICTURE_HASH && nalUnitType != NAL_UNIT_SUFFIX_SEI,
      "When a scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture hash, "
      "the SEI NAL unit containing the scalable nesting SEI message shall have nal_unit_type equal to SUFFIX_SEI_NUT");

    CHECK(nestedsei->payloadType() == SEI::PayloadType::DECODED_PICTURE_HASH && sn.subpicId.empty(),
          "When the scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture "
          "hash, the value of sn_subpic_flag shall be equal to 1");

    CHECK(nestedsei->payloadType() == SEI::PayloadType::SUBPICTURE_LEVEL_INFO && !sn.subpicId.empty(),
          "When the scalable nesting SEI message contains an SEI message that has payloadType equal to SLI, the value "
          "of sn_subpic_flag shall be equal to 0");

    CHECK(generalHrd && generalHrd->getGeneralSamePicTimingInAllOlsFlag()
            && nestedsei->payloadType() == SEI::PayloadType::PICTURE_TIMING,
          "When general_same_pic_timing_in_all_ols_flag is equal to 1, there shall be no SEI NAL unit that contain a "
          "scalable-nested SEI message with payloadType equal to PT");

    for (int i = 0; i < vclAssociatedSeiList.size(); i++)
    {
      CHECK(nestedsei->payloadType() == vclAssociatedSeiList[i] && !sn.olsIdx.empty(),
            "When the scalable nesting SEI message contains an SEI message that has payloadType equal to a value in "
            "vclAssociatedSeiList, the value of sn_ols_flag shall be equal to 0");
    }

    if (nestedsei->payloadType() == SEI::PayloadType::BUFFERING_PERIOD
        || nestedsei->payloadType() == SEI::PayloadType::PICTURE_TIMING
        || nestedsei->payloadType() == SEI::PayloadType::DECODING_UNIT_INFO
        || nestedsei->payloadType() == SEI::PayloadType::SUBPICTURE_LEVEL_INFO)
    {
      containBPorPTorDUIorSLI = true;
      CHECK(sn.olsIdx.empty(),
            "When the scalable nesting SEI message contains an SEI message that has payloadType equal to BP, "
            "PT, or DUI, or SLI, the value of sn_ols_flag shall be equal to 1");
    }
    if (!(nestedsei->payloadType() == SEI::PayloadType::BUFFERING_PERIOD
          || nestedsei->payloadType() == SEI::PayloadType::PICTURE_TIMING
          || nestedsei->payloadType() == SEI::PayloadType::DECODING_UNIT_INFO
          || nestedsei->payloadType() == SEI::PayloadType::SUBPICTURE_LEVEL_INFO))
    {
      containNoBPorPTorDUIorSLI = true;
    }
  }
  CHECK(containBPorPTorDUIorSLI && containNoBPorPTorDUIorSLI, "When a scalable nesting SEI message contains a BP, PT, DUI, or SLI SEI message, the scalable nesting SEI message shall not contain any other SEI message with payloadType not equal to BP, PT, DUI, or SLI");
}

void SEIReader::xParseSEIDecodingUnitInfo(SEIDecodingUnitInfo& dui, uint32_t payloadSize, const SEIBufferingPeriod& bp,
                                          const uint32_t temporalId, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(dui, pDecodedMessageOutputStream, payloadSize);
  sei_read_uvlc(pDecodedMessageOutputStream, val, "dui_decoding_unit_idx");
  dui.decodingUnitIdx = val;

  if (!bp.duCpbParamsInPicTimingSei)
  {
    for (int i = temporalId; i < bp.maxSublayers; i++)
    {
      if (i < bp.maxSublayers - 1)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "dui_sublayer_delays_present_flag[i]");
        dui.hasSublayerDelays[i] = val != 0;
      }
      else
      {
        dui.hasSublayerDelays[i] = true;
      }
      if (dui.hasSublayerDelays[i])
      {
        sei_read_code(pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, val,
                      "dui_du_cpb_removal_delay_increment[i]");
        dui.duCpbRemovalDelayIncrement[i] = val;
      }
      else
      {
        dui.duCpbRemovalDelayIncrement[i] = 0;
      }
    }
  }
  else
  {
    for (int i = temporalId; i < bp.maxSublayers - 1; i++)
    {
      dui.duCpbRemovalDelayIncrement[i] = 0;
    }
  }

  if (!bp.duDpbParamsInPicTimingSei)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "dui_dpb_output_du_delay_present_flag");
    dui.hasDpbOutputDuDelay = (val != 0);
  }
  else
  {
    dui.hasDpbOutputDuDelay = false;
  }
  if (dui.hasDpbOutputDuDelay)
  {
    sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayDuLength, val, "dui_dpb_output_du_delay");
    CHECK(dui.dpbOutputDuDelay != -1 && dui.dpbOutputDuDelay != val,
          "When signaled dpbOutputDuDelay value must be same for DUs");
    dui.dpbOutputDuDelay = val;
  }
}

void SEIReader::xParseSEIBufferingPeriod(SEIBufferingPeriod& bp, uint32_t payloadSize,
                                         std::ostream* pDecodedMessageOutputStream)
{
  uint32_t code;

  output_sei_message_header(bp, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, code, "bp_nal_hrd_params_present_flag");
  bp.hasHrdParams[HrdType::NAL] = code != 0;
  sei_read_flag(pDecodedMessageOutputStream, code, "bp_vcl_hrd_params_present_flag");
  bp.hasHrdParams[HrdType::VCL] = code != 0;

  CHECK(!bp.hasHrdParams[HrdType::NAL] && !bp.hasHrdParams[HrdType::VCL],
        "bp_vcl_hrd_params_present_flag and bp_nal_hrd_params_present_flag in a BP SEI message shall not both be equal "
        "to 0");

  sei_read_code(pDecodedMessageOutputStream, 5, code, "bp_cpb_initial_removal_delay_length_minus1");
  bp.cpbInitialRemovalDelayLength = code + 1;
  sei_read_code(pDecodedMessageOutputStream, 5, code, "bp_cpb_removal_delay_length_minus1");
  bp.cpbRemovalDelayLength = code + 1;
  sei_read_code(pDecodedMessageOutputStream, 5, code, "bp_dpb_output_delay_length_minus1");
  bp.dpbOutputDelayLength = code + 1;
  sei_read_flag(pDecodedMessageOutputStream, code, "bp_du_hrd_params_present_flag");
  bp.hasDuHrdParams = code != 0;
  if (bp.hasDuHrdParams)
  {
    sei_read_code(pDecodedMessageOutputStream, 5, code, "bp_du_cpb_removal_delay_increment_length_minus1");
    bp.duCpbRemovalDelayIncrementLength = code + 1;
    sei_read_code(pDecodedMessageOutputStream, 5, code, "bp_dpb_output_delay_du_length_minus1");
    bp.dpbOutputDelayDuLength = code + 1;
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_du_cpb_params_in_pic_timing_sei_flag");
    bp.duCpbParamsInPicTimingSei = code != 0;
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_du_dpb_params_in_pic_timing_sei_flag");
    bp.duDpbParamsInPicTimingSei = code != 0;
  }
  else
  {
    bp.duCpbRemovalDelayIncrementLength = 24;
    bp.dpbOutputDelayDuLength           = 24;
    bp.duCpbParamsInPicTimingSei        = false;
    bp.duDpbParamsInPicTimingSei        = false;
  }

  sei_read_flag(pDecodedMessageOutputStream, code, "bp_concatenation_flag");
  bp.concatenation = code != 0;
  sei_read_flag(pDecodedMessageOutputStream, code, "bp_additional_concatenation_info_present_flag");
  bp.hasAdditionalConcatenationInfo = code != 0;
  if (bp.hasAdditionalConcatenationInfo)
  {
    sei_read_code(pDecodedMessageOutputStream, bp.cpbInitialRemovalDelayLength, code,
                  "bp_max_initial_removal_delay_for_concatenation");
    bp.maxInitialRemovalDelayForConcatenation = code;
  }

  sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, code, "bp_cpb_removal_delay_delta_minus1");
  bp.cpbRemovalDelayDelta = code + 1;
  sei_read_code(pDecodedMessageOutputStream, 3, code, "bp_max_sublayers_minus1");
  bp.maxSublayers = code + 1;

  bp.cpbRemovalDelayDeltaVals.clear();

  if (bp.maxSublayers > 1)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_cpb_removal_delay_deltas_present_flag");

    if (code != 0)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, code, "bp_num_cpb_removal_delay_deltas_minus1");
      CHECK(code >= bp.cpbRemovalDelayDeltaVals.capacity(),
            "The value of num_cpb_removal_offsets_minus1 shall be in the range of 0 to 15, inclusive.")
      bp.cpbRemovalDelayDeltaVals.resize(code + 1);
      for (size_t i = 0; i < bp.numCpbRemovalDelayDeltas(); i++)
      {
        sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, code, "bp_cpb_removal_delay_delta_val[i]");
        bp.cpbRemovalDelayDeltaVals[i] = code;
      }
    }
  }

  sei_read_uvlc(pDecodedMessageOutputStream, code, "bp_cpb_cnt_minus1");
  CHECK(code >= MAX_CPB_CNT, "The value of bp_cpb_cnt_minus1 shall be in the range of 0 to 31, inclusive");

  bp.cpbCount = code + 1;
  if (bp.maxSublayers > 1)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_initial_cpb_removal_delay_present_flag");
    bp.hasSublayerInitialCpbRemovalDelay = code != 0;
  }
  else
  {
    bp.hasSublayerInitialCpbRemovalDelay = false;
  }

  for (int sublayerIdx = (bp.hasSublayerInitialCpbRemovalDelay ? 0 : bp.maxSublayers - 1);
       sublayerIdx < bp.maxSublayers; sublayerIdx++)
  {
    for (auto hrdType: { HrdType::NAL, HrdType::VCL })
    {
      if (bp.hasHrdParams[hrdType])
      {
        for (int j = 0; j < bp.cpbCount; j++)
        {
          sei_read_code(pDecodedMessageOutputStream, bp.cpbInitialRemovalDelayLength, code,
                        hrdType == HrdType::NAL ? "bp_nal_initial_cpb_removal_delay[i][j]"
                                                : "bp_vcl_initial_cpb_removal_delay[i][j]");
          bp.initialCpbRemoval[hrdType][sublayerIdx][j].delay = code;
          sei_read_code(pDecodedMessageOutputStream, bp.cpbInitialRemovalDelayLength, code,
                        hrdType == HrdType::NAL ? "bp_nal_initial_cpb_removal_offset[i][j]"
                                                : "bp_vcl_initial_cpb_removal_offset[i][j]");
          bp.initialCpbRemoval[hrdType][sublayerIdx][j].offset = code;

          if (bp.hasDuHrdParams)
          {
            sei_read_code(pDecodedMessageOutputStream, bp.cpbInitialRemovalDelayLength, code,
                          hrdType == HrdType::NAL ? "bp_nal_initial_alt_cpb_removal_delay[i][j]"
                                                  : "bp_vcl_initial_alt_cpb_removal_delay[i][j]");
            bp.initialAltCpbRemoval[hrdType][sublayerIdx][j].delay = code;
            sei_read_code(pDecodedMessageOutputStream, bp.cpbInitialRemovalDelayLength, code,
                          hrdType == HrdType::NAL ? "bp_nal_initial_alt_cpb_removal_offset[i][j]"
                                                  : "bp_vcl_initial_alt_cpb_removal_offset[i][j]");
            bp.initialAltCpbRemoval[hrdType][sublayerIdx][j].offset = code;
          }
        }
      }
    }
  }
  if (bp.maxSublayers > 1)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_dpb_output_offsets_present_flag");
    bp.hasSublayerDpbOutputOffsets = code;
  }
  else
  {
    bp.hasSublayerDpbOutputOffsets = false;
  }
  if (bp.hasSublayerDpbOutputOffsets)
  {
    for (int i = 0; i < bp.maxSublayers - 1; i++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, code, "bp_dpb_output_tid_offset[i]");
      bp.dpbOutputTidOffset[i] = code;
    }
    bp.dpbOutputTidOffset[bp.maxSublayers - 1] = 0;
  }
  sei_read_flag(pDecodedMessageOutputStream, code, "bp_alt_cpb_params_present_flag");
  bp.hasAltCpbParams = code;
  if (bp.hasAltCpbParams)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_use_alt_cpb_params_flag");
    bp.useAltCpbParams = code;
  }

  CHECK(bp.hasAltCpbParams && bp.hasDuHrdParams, "When bp_alt_cpb_params_present_flag is equal to 1, the value of "
                                                 "bp_du_hrd_params_present_flag shall be equal to 0");
}

void SEIReader::xParseSEIPictureTiming(SEIPictureTiming& pt, uint32_t payloadSize, const uint32_t temporalId,
                                       const SEIBufferingPeriod& bp, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(pt, pDecodedMessageOutputStream, payloadSize);

  uint32_t symbol;
  sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, symbol,
                "pt_cpb_removal_delay_minus1[bp_max_sub_layers_minus1]");
  pt.cpbRemovalDelay[bp.maxSublayers - 1]   = symbol + 1;
  pt.hasSublayerDelays[bp.maxSublayers - 1] = true;
  for (int i = temporalId; i < bp.maxSublayers - 1; i++)
  {
    sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_sublayer_delays_present_flag[i]");
    pt.hasSublayerDelays[i] = symbol != 0;
    if (pt.hasSublayerDelays[i])
    {
      if (bp.hasCpbRemovalDelayDeltas())
      {
        sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_cpb_removal_delay_delta_enabled_flag[i]");
        pt.cpbRemovalDelayDeltaEnabled[i] = symbol != 0;
      }
      else
      {
        pt.cpbRemovalDelayDeltaEnabled[i] = false;
      }
      if (pt.cpbRemovalDelayDeltaEnabled[i])
      {
        if (bp.numCpbRemovalDelayDeltas() > 1)
        {
          sei_read_code(pDecodedMessageOutputStream, ceilLog2(bp.numCpbRemovalDelayDeltas()), symbol,
                        "pt_cpb_removal_delay_delta_idx[i]");
          pt.cpbRemovalDelayDeltaIdx[i] = symbol;
        }
        else
        {
          pt.cpbRemovalDelayDeltaIdx[i] = 0;
        }
      }
      else
      {
        sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, symbol, "pt_cpb_removal_delay_minus1[i]");
        pt.cpbRemovalDelay[i] = symbol + 1;
      }
    }
  }
  sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayLength, symbol, "pt_dpb_output_delay");
  pt.dpbOutputDelay = symbol;

  if (bp.hasAltCpbParams)
  {
    sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_cpb_alt_timing_info_present_flag");
    pt.hasAltTimingInfo = symbol != 0;
    if (pt.hasAltTimingInfo)
    {
      for (auto hrdType: { HrdType::NAL, HrdType::VCL })
      {
        if (bp.hasHrdParams[hrdType])
        {
          for (int i = (bp.hasSublayerInitialCpbRemovalDelay ? 0 : bp.maxSublayers - 1); i < bp.maxSublayers; ++i)
          {
            for (int j = 0; j < bp.cpbCount; j++)
            {
              sei_read_code(pDecodedMessageOutputStream, bp.cpbInitialRemovalDelayLength, symbol,
                            hrdType == HrdType::NAL ? "pt_nal_cpb_alt_initial_removal_delay_delta[ i ][ j ]"
                                                    : "pt_vcl_cpb_alt_initial_removal_delay_delta[ i ][ j ]");
              pt.initialAltCpbRemovalDelta[hrdType][i][j].delay = symbol;
              sei_read_code(pDecodedMessageOutputStream, bp.cpbInitialRemovalDelayLength, symbol,
                            hrdType == HrdType::NAL ? "pt_nal_cpb_alt_initial_removal_offset_delta[ i ][ j ]"
                                                    : "pt_vcl_cpb_alt_initial_removal_offset_delta[ i ][ j ]");
              pt.initialAltCpbRemovalDelta[hrdType][i][j].offset = symbol;
            }
            sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, pt.cpbDelayOffset[hrdType][i],
                          hrdType == HrdType::NAL ? "pt_nal_cpb_delay_offset[ i ]" : "pt_vcl_cpb_delay_offset[ i ]");
            sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayLength, pt.dpbDelayOffset[hrdType][i],
                          hrdType == HrdType::NAL ? "pt_nal_dpb_delay_offset[ i ]" : "pt_vcl_dpb_delay_offset[ i ]");
          }
        }
      }
    }
  }
  else
  {
    pt.hasAltTimingInfo = false;
  }

  if (bp.hasDuHrdParams && bp.duDpbParamsInPicTimingSei)
  {
    sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayDuLength, symbol, "pt_dpb_output_du_delay");
    pt.dpbOutputDuDelay = symbol;
  }
  if (bp.hasDuHrdParams && bp.duCpbParamsInPicTimingSei)
  {
    sei_read_uvlc(pDecodedMessageOutputStream, symbol, "pt_num_decoding_units_minus1");
    pt.setNumDecodingUnits(symbol + 1);

    if (pt.getNumDecodingUnits() > 1)
    {
      sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_du_common_cpb_removal_delay_flag");
      pt.duCommonCpbRemovalDelay = symbol;
      if (pt.duCommonCpbRemovalDelay)
      {
        for (int i = temporalId; i < bp.maxSublayers; i++)
        {
          if (pt.hasSublayerDelays[i])
          {
            sei_read_code(pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, symbol,
                          "pt_du_common_cpb_removal_delay_increment_minus1[i]");
            pt.duCommonCpbRemovalDelayIncrement[i] = symbol + 1;
          }
        }
      }
      for (int i = 0; i < pt.getNumDecodingUnits(); i++)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, symbol, "pt_num_nalus_in_du_minus1[i]");
        pt.numNalusInDu[i] = symbol + 1;
        if (!pt.duCommonCpbRemovalDelay && i < pt.getNumDecodingUnits() - 1)
        {
          for (int j = temporalId; j < bp.maxSublayers; j++)
          {
            if (pt.hasSublayerDelays[j])
            {
              sei_read_code(pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, symbol,
                            "pt_du_cpb_removal_delay_increment_minus1[i][j]");
              pt.duCpbRemovalDelayIncrement[i][j] = symbol + 1;
            }
          }
        }
      }
    }
    else
    {
      pt.duCommonCpbRemovalDelay = 0;
    }
  }

  if (bp.hasAdditionalConcatenationInfo)
  {
    sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_delay_for_concatenation_ensured_flag");
    pt.delayForConcatenationEnsured = symbol != 0;
  }

  sei_read_code(pDecodedMessageOutputStream, 8, symbol, "pt_display_elemental_periods_minus1");
  pt.displayElementalPeriods = symbol + 1;
}

void SEIReader::xParseSEIAnnotatedRegions(SEIAnnotatedRegions& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag(pDecodedMessageOutputStream, val, "ar_cancel_flag");                                   sei.m_hdr.m_cancelFlag = val;
  if (!sei.m_hdr.m_cancelFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_not_optimized_for_viewing_flag");              sei.m_hdr.m_notOptimizedForViewingFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_true_motion_flag");                            sei.m_hdr.m_trueMotionFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_occluded_object_flag");                        sei.m_hdr.m_occludedObjectFlag = val; // must be constant
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_partial_object_flag_present_flag");            sei.m_hdr.m_partialObjectFlagPresentFlag = val; // must be constant
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_label_present_flag");                   sei.m_hdr.m_objectLabelPresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_confidence_info_present_flag");         sei.m_hdr.m_objectConfidenceInfoPresentFlag = val; // must be constant
    if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 4, val, "ar_object_confidence_length_minus_1"); sei.m_hdr.m_objectConfidenceLength = (val + 1); // must be constant
    }
    if (sei.m_hdr.m_objectLabelPresentFlag)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_label_language_present_flag");      sei.m_hdr.m_objectLabelLanguagePresentFlag = val;
      if (sei.m_hdr.m_objectLabelLanguagePresentFlag)
      {
        // byte alignment
        while (m_pcBitstream->getNumBitsRead() % 8 != 0)
        {
          uint32_t code;
          sei_read_flag(pDecodedMessageOutputStream, code, "ar_bit_equal_to_zero");
        }
        sei.m_hdr.m_annotatedRegionsObjectLabelLang.clear();
        do
        {
          sei_read_code(pDecodedMessageOutputStream, 8, val, "ar_label_language");
          if (val)
          {
            assert(sei.m_hdr.m_annotatedRegionsObjectLabelLang.size()<256);
            sei.m_hdr.m_annotatedRegionsObjectLabelLang.push_back((char)val);
          }
        } while (val != '\0');
      }

      uint32_t numLabelUpdates;
      sei_read_uvlc(pDecodedMessageOutputStream, numLabelUpdates, "ar_num_label_updates");
      assert(numLabelUpdates<256);

      sei.m_annotatedLabels.clear();
      sei.m_annotatedLabels.resize(numLabelUpdates);
      for (auto it=sei.m_annotatedLabels.begin(); it!=sei.m_annotatedLabels.end(); it++)
      {
        SEIAnnotatedRegions::AnnotatedRegionLabel &ar = it->second;
        sei_read_uvlc(pDecodedMessageOutputStream, val, "ar_label_idx[]");             it->first = val;
        assert(val<256);
        sei_read_flag(pDecodedMessageOutputStream, val, "ar_label_cancel_flag");       ar.labelValid = !val;
        if (ar.labelValid)
        {
          ar.label.clear();
          // byte alignment
          while (m_pcBitstream->getNumBitsRead() % 8 != 0)
          {
            uint32_t code;
            sei_read_flag(pDecodedMessageOutputStream, code, "ar_bit_equal_to_zero");
          }
          do
          {
            sei_read_code(pDecodedMessageOutputStream, 8, val, "ar_label[]");
            if (val)
            {
              assert(ar.label.size()<256);
              ar.label.push_back((char)val);
            }
          } while (val != '\0');
        }
      }
    }

    uint32_t numObjUpdates;
    sei_read_uvlc(pDecodedMessageOutputStream, numObjUpdates, "ar_num_object_updates");
    assert(numObjUpdates<256);
    sei.m_annotatedRegions.clear();
    sei.m_annotatedRegions.resize(numObjUpdates);
    for (auto it=sei.m_annotatedRegions.begin(); it!=sei.m_annotatedRegions.end(); it++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "ar_object_idx"); it->first=val;
      assert(val<256);
      SEIAnnotatedRegions::AnnotatedRegionObject &ar = it->second;
      sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_cancel_flag");                           ar.objectCancelFlag = val;
      ar.objectLabelValid=false;
      ar.boundingBoxValid=false;
      ar.boundingBoxCancelFlag=true;

      if (!ar.objectCancelFlag)
      {
        if (sei.m_hdr.m_objectLabelPresentFlag)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_label_update_flag");             ar.objectLabelValid = val;
          if (ar.objectLabelValid)
          {
            sei_read_uvlc(pDecodedMessageOutputStream, val, "ar_object_label_idx");                      ar.objLabelIdx = val;
            assert(val<256);
          }
        }
        sei_read_flag(pDecodedMessageOutputStream, val, "ar_bounding_box_update_flag");              ar.boundingBoxValid = val;
        if (ar.boundingBoxValid)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "ar_bounding_box_cancel_flag");             ar.boundingBoxCancelFlag = val;
          if (!ar.boundingBoxCancelFlag)
          {
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_top");                      ar.boundingBoxTop = val;
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_left");                     ar.boundingBoxLeft = val;
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_width");                    ar.boundingBoxWidth = val;
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_height");                   ar.boundingBoxHeight = val;
            if (sei.m_hdr.m_partialObjectFlagPresentFlag)
            {
              sei_read_flag(pDecodedMessageOutputStream, val, "ar_partial_object_flag");                ar.partialObjectFlag = val;
            }
            if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
            {
              sei_read_code(pDecodedMessageOutputStream, sei.m_hdr.m_objectConfidenceLength, val, "ar_object_confidence"); ar.objectConfidence = val;
            }
          }
        }
      }
    }
  }
}

void SEIReader::xParseSEIFrameFieldinfo(SEIFrameFieldInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "ffi_field_pic_flag" );
  sei.m_fieldPicFlag= symbol;
  if (sei.m_fieldPicFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "ffi_bottom_field_flag" );
    sei.m_bottomFieldFlag = symbol;
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "ffi_pairing_indicated_flag" );
    sei.m_pairingIndicatedFlag = symbol;
    if (sei.m_pairingIndicatedFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "ffi_paired_with_next_field_flag" );
      sei.m_pairedWithNextFieldFlag = symbol;
    }
  }
  else
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "ffi_display_fields_from_frame_flag" );
    sei.m_displayFieldsFromFrameFlag = symbol;
    if (sei.m_displayFieldsFromFrameFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "ffi_top_field_first_flag" );
      sei.m_topFieldFirstFlag = symbol;
    }
    sei_read_code( pDecodedMessageOutputStream, 8, symbol, "ffi_display_elemental_periods_minus1" );
    sei.m_displayElementalPeriodsMinus1 = symbol;
  }
  sei_read_code( pDecodedMessageOutputStream, 2, symbol,   "ffi_source_scan_type" );
  sei.m_sourceScanType = symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "ffi_duplicate_flag" );
  sei.m_duplicateFlag = symbol;
}

void SEIReader::xParseSEIDependentRAPIndication( SEIDependentRAPIndication& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
}


void SEIReader::xParseSEIFramePacking(SEIFramePacking& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, val, "fp_arrangement_id" );                    sei.m_arrangementId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "fp_arrangement_cancel_flag" );           sei.m_arrangementCancelFlag = val;

  if( !sei.m_arrangementCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 7, val, "fp_arrangement_type" );             sei.m_arrangementType = val;
    CHECK( ( sei.m_arrangementType <= 2 ) || ( sei.m_arrangementType >= 6 ), "Invalid arrangement type" );

    sei_read_flag( pDecodedMessageOutputStream, val, "fp_quincunx_sampling_flag" );          sei.m_quincunxSamplingFlag = val;

    sei_read_code( pDecodedMessageOutputStream, 6, val, "fp_content_interpretation_type" );  sei.m_contentInterpretationType = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_spatial_flipping_flag" );           sei.m_spatialFlippingFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_frame0_flipped_flag" );             sei.m_frame0FlippedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_field_views_flag" );                sei.m_fieldViewsFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_current_frame_is_frame0_flag" );    sei.m_currentFrameIsFrame0Flag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_frame0_self_contained_flag" );      sei.m_frame0SelfContainedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_frame1_self_contained_flag" );      sei.m_frame1SelfContainedFlag = val;

    if ( sei.m_quincunxSamplingFlag == 0 && sei.m_arrangementType != 5)
    {
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame0_grid_position_x" );     sei.m_frame0GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame0_grid_position_y" );     sei.m_frame0GridPositionY = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame1_grid_position_x" );     sei.m_frame1GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame1_grid_position_y" );     sei.m_frame1GridPositionY = val;
    }

    sei_read_code( pDecodedMessageOutputStream, 8, val, "fp_arrangement_reserved_byte" );    sei.m_arrangementReservedByte = val;
    sei_read_flag( pDecodedMessageOutputStream, val,  "fp_arrangement_persistence_flag" );   sei.m_arrangementPersistenceFlag = (val != 0);
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "fp_upsampled_aspect_ratio_flag" );       sei.m_upsampledAspectRatio = val;
}

void SEIReader::xParseSEIDisplayOrientation(SEIDisplayOrientation& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, val, "display_orientation_cancel_flag");           sei.m_doCancelFlag = val;
  if (!sei.m_doCancelFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "display_orientation_persistence_flag");    sei.m_doPersistenceFlag = val;
    sei_read_code(pDecodedMessageOutputStream, 3, val, "display_orientation_transform_type");   sei.m_doTransformType = val;
    CHECK((sei.m_doTransformType < 0) || (sei.m_doTransformType > 7), "Invalid transform type");
  }
}

void SEIReader::xParseSEIParameterSetsInclusionIndication(SEIParameterSetsInclusionIndication& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header( sei, pDecodedMessageOutputStream, payloadSize );

  sei_read_flag( pDecodedMessageOutputStream, val, "psii_self_contained_clvs_flag" );
  sei.m_selfContainedClvsFlag = val;
}

void SEIReader::xParseSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_x[0]" ); sei.values.primaries[0][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_y[0]" ); sei.values.primaries[0][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_x[1]" ); sei.values.primaries[1][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_y[1]" ); sei.values.primaries[1][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_x[2]" ); sei.values.primaries[2][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_y[2]" ); sei.values.primaries[2][1] = code;


  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_white_point_x" ); sei.values.whitePoint[0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_white_point_y" ); sei.values.whitePoint[1] = code;

  sei_read_code( pDecodedMessageOutputStream, 32, code, "mdcv_max_display_mastering_luminance" ); sei.values.maxLuminance = code;
  sei_read_code( pDecodedMessageOutputStream, 32, code, "mdcv_min_display_mastering_luminance" ); sei.values.minLuminance = code;
}

void SEIReader::xParseSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 8, code, "preferred_transfer_characteristics"); sei.m_preferredTransferCharacteristics = code;
}
void SEIReader::xParseSEIUserDataRegistered(SEIUserDataRegistered& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t code;
  assert(payloadSize>0);
  sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code"); payloadSize--;
  if (code == 255)
  {
    assert(payloadSize>0);
    sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code_extension_byte"); payloadSize--;
    code += 255;
  }
  sei.m_ituCountryCode = code;
  sei.m_userData.resize(payloadSize);
  for (uint32_t i = 0; i < sei.m_userData.size(); i++)
  {
    sei_read_code(nullptr, 8, code, "itu_t_t35_payload_byte");
    sei.m_userData[i] = code;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  itu_t_t35 payload size: " << sei.m_userData.size() << "\n";
  }
}

void SEIReader::xParseSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, code, "fg_characteristics_cancel_flag");                sei.m_filmGrainCharacteristicsCancelFlag = code != 0;
  if (!sei.m_filmGrainCharacteristicsCancelFlag)
  {
    sei_read_code(pDecodedMessageOutputStream, 2, code, "fg_model_id");                              sei.m_filmGrainModelId = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "fg_separate_colour_description_present_flag"); sei.m_separateColourDescriptionPresentFlag = code != 0;
    if (sei.m_separateColourDescriptionPresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 3, code, "fg_bit_depth_luma_minus8");               sei.m_filmGrainBitDepthLumaMinus8 = code;
      sei_read_code(pDecodedMessageOutputStream, 3, code, "fg_bit_depth_chroma_minus8");             sei.m_filmGrainBitDepthChromaMinus8 = code;
      sei_read_flag(pDecodedMessageOutputStream, code, "fg_full_range_flag");                        sei.m_filmGrainFullRangeFlag = code != 0;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_colour_primaries");                    sei.m_filmGrainColourPrimaries = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_transfer_characteristics");            sei.m_filmGrainTransferCharacteristics = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_matrix_coeffs");                       sei.m_filmGrainMatrixCoeffs = code;
    }
    sei_read_code(pDecodedMessageOutputStream, 2, code, "fg_blending_mode_id");                      sei.m_blendingModeId = code;
    sei_read_code(pDecodedMessageOutputStream, 4, code, "fg_log2_scale_factor");                     sei.m_log2ScaleFactor = code;
    for (int c = 0; c<3; c++)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "fg_comp_model_present_flag[c]");             sei.m_compModel[c].presentFlag = code != 0;
    }
    for (int c = 0; c<3; c++)
    {
      SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      if (cm.presentFlag)
      {
        sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_num_intensity_intervals_minus1[c]"); cm.numIntensityIntervals = code + 1;
        sei_read_code(pDecodedMessageOutputStream, 3, code, "fg_num_model_values_minus1[c]");        cm.numModelValues = code + 1;
        cm.intensityValues.resize(cm.numIntensityIntervals);
        for (uint32_t interval = 0; interval < cm.numIntensityIntervals; interval++)
        {
          SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv = cm.intensityValues[interval];
          sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_intensity_interval_lower_bound[c][i]"); cmiv.intensityIntervalLowerBound = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_intensity_interval_upper_bound[c][i]"); cmiv.intensityIntervalUpperBound = code;
          cmiv.compModelValue.resize(cm.numModelValues);
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            sei_read_svlc(pDecodedMessageOutputStream, cmiv.compModelValue[j], "fg_comp_model_value[c][i]");
          }
        }
      }
    } // for c
    sei_read_flag(pDecodedMessageOutputStream, code, "fg_characteristics_persistence_flag");         sei.m_filmGrainCharacteristicsPersistenceFlag = code != 0;
  } // cancel flag
}

void SEIReader::xParseSEIContentLightLevelInfo(SEIContentLightLevelInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 16, code, "clli_max_content_light_level");     sei.m_maxContentLightLevel = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "clli_max_pic_average_light_level"); sei.m_maxPicAverageLightLevel = code;
}

void SEIReader::xParseSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 32, code, "ambient_illuminance"); sei.m_ambientIlluminance = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_x");     sei.m_ambientLightX = (uint16_t)code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_y");     sei.m_ambientLightY = (uint16_t)code;
}

void SEIReader::xParseSEIColourTransformInfo(SEIColourTransformInfo& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t code;

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc(pDecodedMessageOutputStream, code, "colour_transform_id");               sei.m_id = code;
  sei_read_flag(pDecodedMessageOutputStream, code, "colour_transform_cancel_flag");      bool colourTransformCancelFlag = code;

  if (colourTransformCancelFlag == 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "colour_transform_persistence_flag");
    sei_read_flag(pDecodedMessageOutputStream, code, "colour_transform_video_signal_info_present_flag"); sei.m_signalInfoFlag = code;

    if (sei.m_signalInfoFlag)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "colour_transform_full_range_flag");        sei.m_fullRangeFlag = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "colour_transform_primaries");           sei.m_primaries = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "colour_transform_transfer_function");   sei.m_transferFunction = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "colour_transform_matrix_coefficients"); sei.m_matrixCoefs = code;
    }
    else
    {
      sei.m_fullRangeFlag = 0;
      sei.m_primaries = 0;
      sei.m_transferFunction = 0;
      sei.m_matrixCoefs = 0;
    }
    sei_read_code(pDecodedMessageOutputStream, 4, code, "colour_transform_bit_depth_minus8");                       sei.m_bitdepth = 8+code;
    sei_read_code(pDecodedMessageOutputStream, 3, code, "colour_transform_log2_number_of_points_per_lut_minus1");   sei.m_log2NumberOfPointsPerLut = code + 1;
    int numLutValues = (1 << sei.m_log2NumberOfPointsPerLut) + 1;
    sei_read_flag(pDecodedMessageOutputStream, code, "colour_transform_cross_comp_flag");                sei.m_crossComponentFlag = code;
    sei.m_crossComponentInferred = 0;
    if (sei.m_crossComponentFlag == true)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "colour_transform_cross_comp_inferred");          sei.m_crossComponentInferred = code;
    }
    for (int i = 0; i < MAX_NUM_COMPONENT; i++) {
      sei.m_lut[i].lutValues.resize(numLutValues);
    }

    uint16_t lutCodingLength = 2 + sei.m_bitdepth - sei.m_log2NumberOfPointsPerLut;
    for (uint32_t j = 0; j < numLutValues; j++)
    {
      sei_read_code(pDecodedMessageOutputStream, lutCodingLength, code, "colour_transform_lut[0][i]");
      sei.m_lut[0].lutValues[j] = code;
    }
    sei.m_lut[0].numLutValues = numLutValues;
    sei.m_lut[0].presentFlag = true;
    if (sei.m_crossComponentFlag == 0 || sei.m_crossComponentInferred == 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "colour_transform_number_chroma_lut_minus1");      sei.m_numberChromaLutMinus1 = code;
      for (uint32_t j = 0; j < numLutValues; j++)
      {
        sei_read_code(pDecodedMessageOutputStream, lutCodingLength, code, "colour_transform_lut[1][i]");
        sei.m_lut[1].lutValues[j] = code;
        sei.m_lut[2].lutValues[j] = code;
      }
      if (sei.m_numberChromaLutMinus1 == 1)
      {
        for (uint32_t j = 0; j < numLutValues; j++)
        {
          sei_read_code(pDecodedMessageOutputStream, lutCodingLength, code, "colour_transform_lut[2][i]");
          sei.m_lut[2].lutValues[j] = code;
        }
      }
      sei.m_lut[1].numLutValues = numLutValues;
      sei.m_lut[2].numLutValues = numLutValues;
      sei.m_lut[1].presentFlag = true;
      sei.m_lut[2].presentFlag = true;
    }
    else
    {
      sei_read_code(pDecodedMessageOutputStream, lutCodingLength, code, "colour_transform_chroma_offset");
      sei.m_chromaOffset = code;
    }
  }
}

void SEIReader::xParseSEIContentColourVolume(SEIContentColourVolume& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int i;
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, val, "ccv_cancel_flag");          sei.m_ccvCancelFlag = val;
  if (!sei.m_ccvCancelFlag)
  {
    int iVal;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_persistence_flag");   sei.m_ccvPersistenceFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_primaries_present_flag");   sei.m_ccvPrimariesPresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_min_luminance_value_present_flag");   sei.m_ccvMinLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_max_luminance_value_present_flag");   sei.m_ccvMaxLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_avg_luminance_value_present_flag");   sei.m_ccvAvgLuminanceValuePresentFlag = val;

    if (sei.m_ccvPrimariesPresentFlag)
    {
      for (i = 0; i < MAX_NUM_COMPONENT; i++)
      {
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_x[i]");          sei.m_ccvPrimariesX[i] = iVal;
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_y[i]");          sei.m_ccvPrimariesY[i] = iVal;
      }
    }
    if (sei.m_ccvMinLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_min_luminance_value");   sei.m_ccvMinLuminanceValue = val;
    }
    if (sei.m_ccvMaxLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_max_luminance_value");   sei.m_ccvMaxLuminanceValue = val;
    }
    if (sei.m_ccvAvgLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_avg_luminance_value");   sei.m_ccvAvgLuminanceValue = val;
    }
  }
}

void SEIReader::xParseSEIEquirectangularProjection(SEIEquirectangularProjection& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, val,       "erp_cancel_flag" );              sei.m_erpCancelFlag = val;
  if( !sei.m_erpCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_persistence_flag"    );     sei.m_erpPersistenceFlag   = val;
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_guard_band_flag"     );     sei.m_erpGuardBandFlag     = val;
    sei_read_code( pDecodedMessageOutputStream, 2, val,   "erp_reserved_zero_2bits" );
    if ( sei.m_erpGuardBandFlag == 1)
    {
      sei_read_code( pDecodedMessageOutputStream, 3, val,     "erp_guard_band_type"       );   sei.m_erpGuardBandType  = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_left_guard_band_width" );   sei.m_erpLeftGuardBandWidth = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_right_guard_band_width");   sei.m_erpRightGuardBandWidth = val;
    }
  }
}

void SEIReader::xParseSEISphereRotation(SEISphereRotation& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  int  sval;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, val,           "sphere_rotation_cancel_flag" );             sei.m_sphereRotationCancelFlag = val;
  if( !sei.m_sphereRotationCancelFlag )
  {
    sei_read_flag ( pDecodedMessageOutputStream,      val,   "sphere_rotation_persistence_flag"    );     sei.m_sphereRotationPersistenceFlag = val;
    sei_read_code ( pDecodedMessageOutputStream, 6,   val,   "sphere_rotation_reserved_zero_6bits" );
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_yaw"                 );     sei.m_sphereRotationYaw = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_pitch"               );     sei.m_sphereRotationPitch = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_roll"                );     sei.m_sphereRotationRoll = sval;
  }
}

void SEIReader::xParseSEIOmniViewport(SEIOmniViewport& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  int  scode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code( pDecodedMessageOutputStream, 10, code,        "omni_viewport_id"          );       sei.m_omniViewportId         = code;
  sei_read_flag( pDecodedMessageOutputStream,     code,        "omni_viewport_cancel_flag" );       sei.m_omniViewportCancelFlag = code;
  if (!sei.m_omniViewportCancelFlag)
  {
    uint32_t numRegions;
    sei_read_flag( pDecodedMessageOutputStream,    code,       "omni_viewport_persistence_flag" );  sei.m_omniViewportPersistenceFlag = code;
    sei_read_code( pDecodedMessageOutputStream, 4, numRegions, "omni_viewport_cnt_minus1"       );  numRegions++;
    sei.m_omniViewportRegions.resize(numRegions);
    for(uint32_t region=0; region<numRegions; region++)
    {
      SEIOmniViewport::OmniViewport &viewport = sei.m_omniViewportRegions[region];
      sei_read_scode( pDecodedMessageOutputStream, 32, scode,  "omni_viewport_azimuth_centre"   );  viewport.azimuthCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode,  "omni_viewport_elevation_centre" );  viewport.elevationCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode,  "omni_viewport_tilt_centre"      );  viewport.tiltCentre = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code,   "omni_viewport_hor_range"         ); viewport.horRange        = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code,   "omni_viewport_ver_range"         ); viewport.verRange        = code;
    }
  }
  else
  {
    sei.m_omniViewportRegions.clear();
    sei.m_omniViewportPersistenceFlag=false;
  }
}

void SEIReader::xParseSEIRegionWisePacking(SEIRegionWisePacking& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag( pDecodedMessageOutputStream,           val,      "rwp_cancel_flag" );                       sei.m_rwpCancelFlag = val;
  if (!sei.m_rwpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_persistence_flag" );                  sei.m_rwpPersistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_constituent_picture_matching_flag" ); sei.m_constituentPictureMatchingFlag = val;
    sei_read_code( pDecodedMessageOutputStream,       5,  val,    "rwp_reserved_zero_5bits" );
    sei_read_code( pDecodedMessageOutputStream,       8,  val,    "rwp_num_packed_regions" );                sei.m_numPackedRegions = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "rwp_proj_picture_width" );                sei.m_projPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "rwp_proj_picture_height" );               sei.m_projPictureHeight = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "rwp_packed_picture_width" );              sei.m_packedPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "rwp_packed_picture_height" );             sei.m_packedPictureHeight = val;

    sei.m_rwpTransformType.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandFlag.resize(sei.m_numPackedRegions);
    sei.m_projRegionWidth.resize(sei.m_numPackedRegions);
    sei.m_projRegionHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpProjRegionTop.resize(sei.m_numPackedRegions);
    sei.m_projRegionLeft.resize(sei.m_numPackedRegions);
    sei.m_packedRegionWidth.resize(sei.m_numPackedRegions);
    sei.m_packedRegionHeight.resize(sei.m_numPackedRegions);
    sei.m_packedRegionTop.resize(sei.m_numPackedRegions);
    sei.m_packedRegionLeft.resize(sei.m_numPackedRegions);
    sei.m_rwpLeftGuardBandWidth.resize(sei.m_numPackedRegions);
    sei.m_rwpRightGuardBandWidth.resize(sei.m_numPackedRegions);
    sei.m_rwpTopGuardBandHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpBottomGuardBandHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandNotUsedForPredFlag.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandType.resize(4*sei.m_numPackedRegions);

    for( int i=0; i < sei.m_numPackedRegions; i++ )
    {
      sei_read_code( pDecodedMessageOutputStream,     4,  val,    "rwp_reserved_zero_4bits" );
      sei_read_code( pDecodedMessageOutputStream,     3,  val,    "rwp_transform_type" );                    sei.m_rwpTransformType[i] = val;
      sei_read_flag( pDecodedMessageOutputStream,         val,    "rwp_guard_band_flag" );                   sei.m_rwpGuardBandFlag[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_width" );                 sei.m_projRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_height" );                sei.m_projRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_top" );                   sei.m_rwpProjRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_left" );                  sei.m_projRegionLeft[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_width" );               sei.m_packedRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_height" );              sei.m_packedRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_top" );                 sei.m_packedRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_left" );                sei.m_packedRegionLeft[i] = val;
      if( sei.m_rwpGuardBandFlag[i] )
      {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_left_guard_band_width" );             sei.m_rwpLeftGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_right_guard_band_width" );            sei.m_rwpRightGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_top_guard_band_height" );             sei.m_rwpTopGuardBandHeight[i]  = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_bottom_guard_band_height" );          sei. m_rwpBottomGuardBandHeight[i]  = val;
        sei_read_flag( pDecodedMessageOutputStream,       val,    "rwp_guard_band_not_used_for_pred_flag" ); sei.m_rwpGuardBandNotUsedForPredFlag[i] = val;
        for( int j=0; j < 4; j++ )
        {
          sei_read_code( pDecodedMessageOutputStream, 3,  val,    "rwp_guard_band_type" );                   sei.m_rwpGuardBandType[i*4 + j] = val;
        }
        sei_read_code( pDecodedMessageOutputStream,   3,  val,    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

void SEIReader::xParseSEIGeneralizedCubemapProjection(SEIGeneralizedCubemapProjection& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream,          val,    "gcmp_cancel_flag" );                      sei.m_gcmpCancelFlag = val;
  if (!sei.m_gcmpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_persistence_flag"    );              sei.m_gcmpPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     3, val,    "gcmp_packing_type" );                     sei.m_gcmpPackingType = val;
    sei_read_code( pDecodedMessageOutputStream,     2, val,    "gcmp_mapping_function_type"     );        sei.m_gcmpMappingFunctionType = val;

    int numFace = sei.m_gcmpPackingType == 4 || sei.m_gcmpPackingType == 5 ? 5 : 6;
    sei.m_gcmpFaceIndex.resize(numFace);
    sei.m_gcmpFaceRotation.resize(numFace);
    if (sei.m_gcmpMappingFunctionType == 2)
    {
      sei.m_gcmpFunctionCoeffU.resize(numFace);
      sei.m_gcmpFunctionUAffectedByVFlag.resize(numFace);
      sei.m_gcmpFunctionCoeffV.resize(numFace);
      sei.m_gcmpFunctionVAffectedByUFlag.resize(numFace);
    }

    for (int i = 0; i < numFace; i++)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_face_index" );                       sei.m_gcmpFaceIndex[i] = val;
      sei_read_code( pDecodedMessageOutputStream,   2, val,    "gcmp_face_rotation" );                    sei.m_gcmpFaceRotation[i] = val;
      if (sei.m_gcmpMappingFunctionType == 2)
      {
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_u" );                 sei.m_gcmpFunctionCoeffU[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_u_affected_by_v_flag"    ); sei.m_gcmpFunctionUAffectedByVFlag[i] = val;
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_v" );                 sei.m_gcmpFunctionCoeffV[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_v_affected_by_u_flag"    ); sei.m_gcmpFunctionVAffectedByUFlag[i] = val;
      }
    }
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_guard_band_flag" );                  sei.m_gcmpGuardBandFlag = val;
    if (sei.m_gcmpGuardBandFlag)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_guard_band_type" );                   sei.m_gcmpGuardBandType = val;
      sei_read_flag( pDecodedMessageOutputStream,      val,    "gcmp_guard_band_boundary_exterior_flag" ); sei.m_gcmpGuardBandBoundaryExteriorFlag = val;
      sei_read_code( pDecodedMessageOutputStream,   4, val,    "gcmp_guard_band_samples_minus1" );         sei.m_gcmpGuardBandSamplesMinus1 = val;
    }
  }
}

void SEIReader::xParseSEIScalabilityDimensionInfo(SEIScalabilityDimensionInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream,   6,  val,    "sdi_max_layers_minus1" );            sei.m_sdiMaxLayersMinus1 = val;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "sdi_multiview_info_flag" );          sei.m_sdiMultiviewInfoFlag = val;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "sdi_auxiliary_info_flag" );          sei.m_sdiAuxiliaryInfoFlag = val;
  if (sei.m_sdiMultiviewInfoFlag || sei.m_sdiAuxiliaryInfoFlag)
  {
    if (sei.m_sdiMultiviewInfoFlag)
    {
      sei_read_code( pDecodedMessageOutputStream, 4, val, "sdi_view_id_len_minus1" ); sei.m_sdiViewIdLenMinus1 = val;
    }
    for (int i = 0; i <= sei.m_sdiMaxLayersMinus1; i++)
    {
      sei.m_sdiLayerId.resize(sei.m_sdiMaxLayersMinus1 + 1);
      sei_read_code( pDecodedMessageOutputStream, 6, val, "sdi_layer_id" ); sei.m_sdiLayerId[i] = val;
      if (sei.m_sdiMultiviewInfoFlag)
      {
        sei.m_sdiViewIdVal.resize(sei.m_sdiMaxLayersMinus1 + 1);
        sei_read_code( pDecodedMessageOutputStream, sei.m_sdiViewIdLenMinus1 + 1, val, "sdi_view_id_val" ); sei.m_sdiViewIdVal[i] = val;
      }
      if (sei.m_sdiAuxiliaryInfoFlag)
      {
        sei.m_sdiAuxId.resize(sei.m_sdiMaxLayersMinus1 + 1);
        sei.m_sdiNumAssociatedPrimaryLayersMinus1.resize(sei.m_sdiMaxLayersMinus1 + 1);
        sei.m_sdiAssociatedPrimaryLayerIdx.resize(sei.m_sdiMaxLayersMinus1 + 1);
        sei_read_code( pDecodedMessageOutputStream, 8, val, "sdi_aux_id" ); sei.m_sdiAuxId[i] = val;
        if (sei.m_sdiAuxId[i] > 0)
        {
          sei_read_code( pDecodedMessageOutputStream, 6, val, "sdi_num_associated_primary_layers_minus1" ); sei.m_sdiNumAssociatedPrimaryLayersMinus1[i] = val;
          sei.m_sdiAssociatedPrimaryLayerIdx[i].resize(sei.m_sdiNumAssociatedPrimaryLayersMinus1[i] + 1);
          for (int j = 0; j <= sei.m_sdiNumAssociatedPrimaryLayersMinus1[i]; j++)
          {
            sei_read_code( pDecodedMessageOutputStream, 6, val, "sdi_associated_primary_layer_idx" );
            sei.m_sdiAssociatedPrimaryLayerIdx[i][j] = val;
          }
        }
      }
    }
    sei.m_sdiNumViews = 1;
    if (sei.m_sdiMultiviewInfoFlag)
    {
      for (int i = 1; i <= sei.m_sdiMaxLayersMinus1; i++)
      {
        bool newViewFlag = true;
        for (int j = 0; j < i; j++)
        {
          if (sei.m_sdiViewIdVal[i] == sei.m_sdiViewIdVal[j])
          {
            newViewFlag = false;
          }
        }
        if (newViewFlag)
        {
          sei.m_sdiNumViews++;
        }
      }
    }
  }
}

void SEIReader::xParseSEIMultiviewAcquisitionInfo(SEIMultiviewAcquisitionInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag( pDecodedMessageOutputStream, val, "intrinsic_param_flag" ); sei.m_maiIntrinsicParamFlag = (val == 1);
  sei_read_flag( pDecodedMessageOutputStream, val, "extrinsic_param_flag" ); sei.m_maiExtrinsicParamFlag = (val == 1);
  sei_read_uvlc( pDecodedMessageOutputStream, val, "num_views_minus1"     ); sei.m_maiNumViewsMinus1     =  val      ;
  if( sei.m_maiIntrinsicParamFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val, "intrinsic_params_equal_flag" ); sei.m_maiIntrinsicParamsEqualFlag = (val == 1);
    sei.resizeArrays( );
    sei_read_uvlc( pDecodedMessageOutputStream, val, "prec_focal_length"           ); sei.m_maiPrecFocalLength          =  val      ;
    sei_read_uvlc( pDecodedMessageOutputStream, val, "prec_principal_point"        ); sei.m_maiPrecPrincipalPoint       =  val      ;
    sei_read_uvlc( pDecodedMessageOutputStream, val, "prec_skew_factor"            ); sei.m_maiPrecSkewFactor           =  val      ;

    for( int i = 0; i  <=  ( sei.m_maiIntrinsicParamsEqualFlag ? 0 : sei.m_maiNumViewsMinus1 ); i++ )
    {
      sei_read_flag( pDecodedMessageOutputStream,                                         val, "sign_focal_length_x"        ); sei.m_maiSignFocalLengthX       [i] = (val == 1);
      sei_read_code( pDecodedMessageOutputStream, 6,                                      val, "exponent_focal_length_x"    ); sei.m_maiExponentFocalLengthX   [i] =  val      ;
      if (sei.getMantissaFocalLengthXLen( i ) != 0)
      {
        sei_read_code( pDecodedMessageOutputStream, sei.getMantissaFocalLengthXLen( i ),  val, "mantissa_focal_length_x" );
        sei.m_maiMantissaFocalLengthX[i] = val;
      }
      else
      {
        sei.m_maiMantissaFocalLengthX[i] = 0;
      }
      sei_read_flag( pDecodedMessageOutputStream,                                         val, "sign_focal_length_y"        ); sei.m_maiSignFocalLengthY       [i] = (val == 1);
      sei_read_code( pDecodedMessageOutputStream, 6,                                      val, "exponent_focal_length_y"    ); sei.m_maiExponentFocalLengthY   [i] =  val      ;
      if (sei.getMantissaFocalLengthYLen( i ) != 0)
      {
        sei_read_code( pDecodedMessageOutputStream, sei.getMantissaFocalLengthYLen( i ),  val, "mantissa_focal_length_y");
        sei.m_maiMantissaFocalLengthY[i] = val;
      }
      else
      {
        sei.m_maiMantissaFocalLengthY [i] = 0;
      }
      sei_read_flag( pDecodedMessageOutputStream,                                         val, "sign_principal_point_x"     ); sei.m_maiSignPrincipalPointX    [i] = (val == 1);
      sei_read_code( pDecodedMessageOutputStream, 6,                                      val, "exponent_principal_point_x" ); sei.m_maiExponentPrincipalPointX[i] =  val      ;
      if (sei.getMantissaPrincipalPointXLen( i ) != 0)
      {
        sei_read_code( pDecodedMessageOutputStream, sei.getMantissaPrincipalPointXLen( i ), val, "mantissa_principal_point_x" );
        sei.m_maiMantissaPrincipalPointX[i] = val;
      }
      else
      {
        sei.m_maiMantissaPrincipalPointX[i] = 0;
      }
      sei_read_flag( pDecodedMessageOutputStream,                                         val, "sign_principal_point_y"     ); sei.m_maiSignPrincipalPointY    [i] = (val == 1);
      sei_read_code( pDecodedMessageOutputStream, 6,                                      val, "exponent_principal_point_y" ); sei.m_maiExponentPrincipalPointY[i] =  val      ;
      if (sei.getMantissaPrincipalPointYLen( i ) != 0)
      {
        sei_read_code( pDecodedMessageOutputStream, sei.getMantissaPrincipalPointYLen( i ), val, "mantissa_principal_point_y" );
        sei.m_maiMantissaPrincipalPointY[i] = val;
      }
      else
      {
        sei.m_maiMantissaPrincipalPointY[i] = 0;
      }
      sei_read_flag( pDecodedMessageOutputStream,                                         val, "sign_skew_factor"           ); sei.m_maiSignSkewFactor         [i] = (val == 1);
      sei_read_code( pDecodedMessageOutputStream, 6,                                      val, "exponent_skew_factor"       ); sei.m_maiExponentSkewFactor     [i] =  val      ;
      if (sei.getMantissaSkewFactorLen( i ) != 0)
      {
        sei_read_code( pDecodedMessageOutputStream, sei.getMantissaSkewFactorLen( i ),    val, "mantissa_skew_factor" );
        sei.m_maiMantissaSkewFactor[i] = val;
      }
      else
      {
        sei.m_maiMantissaSkewFactor[i] = 0;
      }
    }
  }
  if( sei.m_maiExtrinsicParamFlag )
  {
    sei_read_uvlc( pDecodedMessageOutputStream, val, "prec_rotation_param"    ); sei.m_maiPrecRotationParam    = val;
    sei_read_uvlc( pDecodedMessageOutputStream, val, "prec_translation_param" ); sei.m_maiPrecTranslationParam = val;

    for( int i = 0; i  <=  sei.m_maiNumViewsMinus1; i++ )
    {
      for( int j = 0; j  <=  2; j++ )  /* row */
      {
        for( int k = 0; k  <=  2; k++ )  /* column */
        {
          sei_read_flag( pDecodedMessageOutputStream,                                 val, "sign_r"     ); sei.m_maiSignR    [i][j][k] = (val == 1);
          sei_read_code( pDecodedMessageOutputStream, 6,                              val, "exponent_r" ); sei.m_maiExponentR[i][j][k] =  val      ;
          if (sei.getMantissaRLen( i, j, k ) != 0)
          {
            sei_read_code( pDecodedMessageOutputStream, sei.getMantissaRLen( i, j, k ), val, "mantissa_r" );
            sei.m_maiMantissaR[i][j][k] = val;
          }
          else
          {
            sei.m_maiMantissaR[i][j][k] = 0;
          }
        }
        sei_read_flag( pDecodedMessageOutputStream,                              val, "sign_t"     ); sei.m_maiSignT    [i][j] = (val == 1);
        sei_read_code( pDecodedMessageOutputStream, 6,                           val, "exponent_t" ); sei.m_maiExponentT[i][j] =  val      ;
        if (sei.getMantissaTLen( i, j ) != 0)
        {
          sei_read_code( pDecodedMessageOutputStream, sei.getMantissaTLen( i, j ), val, "mantissa_t" );
          sei.m_maiMantissaT[i][j] = val;
        }
        else
        {
          sei.m_maiMantissaT[i][j] = 0;
        }
      }
    }
  }
}

void SEIReader::xParseSEIMultiviewViewPosition(SEIMultiviewViewPosition& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc(pDecodedMessageOutputStream, val, "num_views_minus1"); sei.m_mvpNumViewsMinus1 = val;
  sei.m_mvpViewPosition.resize(sei.m_mvpNumViewsMinus1 + 1);
  for (int i = 0; i <= sei.m_mvpNumViewsMinus1; i++)
  {
    sei_read_uvlc(pDecodedMessageOutputStream, val, "view_position"); sei.m_mvpViewPosition[i] = val;
  }
}

void SEIReader::xParseSEIAlphaChannelInfo(SEIAlphaChannelInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, val, "alpha_channel_cancel_flag" ); sei.m_aciCancelFlag = (val == 1);
  if( !sei.m_aciCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 3, val, "alpha_channel_use_idc" ); sei.m_aciUseIdc = val;
    sei_read_code( pDecodedMessageOutputStream, 3, val, "alpha_channel_bit_depth_minus8" ); sei.m_aciBitDepthMinus8 = val;
    sei_read_code( pDecodedMessageOutputStream, sei.m_aciBitDepthMinus8 + 9, val, "alpha_transparent_value" ); sei.m_aciTransparentValue = val;
    sei_read_code( pDecodedMessageOutputStream, sei.m_aciBitDepthMinus8 + 9, val, "alpha_opaque_value" ); sei.m_aciOpaqueValue = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "alpha_channel_incr_flag" ); sei.m_aciIncrFlag = (val == 1);
    sei_read_flag( pDecodedMessageOutputStream, val, "alpha_channel_clip_flag" ); sei.m_aciClipFlag = (val == 1);
    if( sei.m_aciClipFlag )
    {
      sei_read_flag( pDecodedMessageOutputStream, val, "alpha_channel_clip_type_flag" ); sei.m_aciClipTypeFlag = (val == 1);
    }
  }
}

void SEIReader::xParseSEIDepthRepresentationInfo(SEIDepthRepresentationInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  double zNear,zFar,dMin,dMax;
  std::vector<int> DepthNonlinearRepresentationModel;

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, val, "z_near_flag" );    sei.m_driZNearFlag  = (val == 1);
  sei_read_flag( pDecodedMessageOutputStream, val, "z_far_flag" );     sei.m_driZFarFlag = (val == 1);
  sei_read_flag( pDecodedMessageOutputStream, val, "d_min_flag" );     sei.m_driDMinFlag = (val == 1);
  sei_read_flag( pDecodedMessageOutputStream, val, "d_max_flag" );     sei.m_driDMaxFlag = (val == 1);
  sei_read_uvlc( pDecodedMessageOutputStream, val, "depth_representation_type" ); sei.m_driDepthRepresentationType = val;
  if( sei.m_driDMinFlag  ||  sei.m_driDMaxFlag )
  {
    sei_read_uvlc( pDecodedMessageOutputStream, val, "disparity_ref_view_id" ); sei.m_driDisparityRefViewId = val;
  }
  if( sei.m_driZNearFlag )
  {
    xParseSEIDepthRepInfoElement(zNear, pDecodedMessageOutputStream);
    sei.m_driZNear = zNear;
  }
  if( sei.m_driZFarFlag )
  {
    xParseSEIDepthRepInfoElement(zFar, pDecodedMessageOutputStream);
    sei.m_driZFar = zFar;
  }
  if( sei.m_driDMinFlag )
  {
    xParseSEIDepthRepInfoElement(dMin, pDecodedMessageOutputStream);
    sei.m_driDMin = dMin;
  }
  if( sei.m_driDMaxFlag )
  {
    xParseSEIDepthRepInfoElement(dMax, pDecodedMessageOutputStream);
    sei.m_driDMax = dMax;
  }

  if( sei.m_driDepthRepresentationType == 3 )
  {
    sei_read_uvlc( pDecodedMessageOutputStream, val, "depth_nonlinear_representation_num_minus1" ); sei.m_driDepthNonlinearRepresentationNumMinus1 = val;
    for( int i = 1; i <= sei.m_driDepthNonlinearRepresentationNumMinus1 + 1; i++ )
    {
      sei_read_uvlc(pDecodedMessageOutputStream,val,"DepthNonlinearRepresentationModel" ) ;
      sei.m_driDepthNonlinearRepresentationModel.push_back(val);
    }
  }
}

void SEIReader::xParseSEIDepthRepInfoElement(double& f,std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  uint32_t x_sign,x_mantissa_len,x_mantissa;
  int x_exp;

  sei_read_flag(pDecodedMessageOutputStream,     val,"da_sign_flag");  x_sign = val ? 1 : 0 ;
  sei_read_code(pDecodedMessageOutputStream,  7, val, "da_exponent" );         x_exp = val-31;
  sei_read_code(pDecodedMessageOutputStream,  5, val, "da_mantissa_len_minus1" );         x_mantissa_len = val+1;
  sei_read_code(pDecodedMessageOutputStream,  x_mantissa_len, val, "da_mantissa" );         x_mantissa = val;
  if (x_mantissa_len>=16)
  {
    f =1.0 +  (x_mantissa*1.0)/(1u<<(x_mantissa_len-16))/(256.0*256.0 );
  }
  else
  {
    f =1.0 +  (x_mantissa*1.0)/(1u<<x_mantissa_len);
  }
  double m=1.0;
  int i;
  if (x_exp<0)
  {
    for(i=0;i<-x_exp;i++)
    {
      m = m * 2;
    }

    f = f/m;
  }
  else
  {
    for(i=0;i<x_exp;i++)
    {
      m = m * 2;
    }

    f= f * m;
  }
  if (x_sign==1)
  {
    f= -f;
  }
}

void SEIReader::xParseSEISubpictureLevelInfo(SEISubpictureLevelInfo& sli, uint32_t payloadSize,
                                             std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sli, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;
  sei_read_code(pDecodedMessageOutputStream, 3, val, "sli_num_ref_levels_minus1");
  const uint32_t numRefLevels = val + 1;

  sei_read_flag(pDecodedMessageOutputStream, val, "sli_cbr_constraint_flag");
  sli.cbrConstraint = val != 0;

  sei_read_flag(pDecodedMessageOutputStream, val, "sli_explicit_fraction_present_flag");
  const bool explicitFractionPresentFlag = val != 0;

  uint32_t maxSublayers = 1;
  uint32_t numSubpics   = 1;
  if (explicitFractionPresentFlag)
  {
    sei_read_uvlc(pDecodedMessageOutputStream, val, "sli_num_subpics_minus1");
    numSubpics = val + 1;
    sei_read_code(pDecodedMessageOutputStream, 3, val, "sli_max_sublayers_minus1");
    maxSublayers = val + 1;
    sei_read_flag(pDecodedMessageOutputStream, val, "sli_sublayer_info_present_flag");
    sli.hasSublayerInfo = val != 0;
    while (!isByteAligned())
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "sli_alignment_zero_bit");
      CHECK(val != 0, "sli_alignment_zero_bit not equal to zero");
    }
  }

  sli.resize(numRefLevels, maxSublayers, explicitFractionPresentFlag, numSubpics);

  // parsing
  for (int k = sli.hasSublayerInfo ? 0 : sli.maxSublayers() - 1; k < sli.maxSublayers(); k++)
  {
    for (int i = 0; i < sli.numRefLevels(); i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_non_subpic_layers_fraction[i][k]");
      sli.nonSubpicLayerFraction(i, k) = (Level::Name) val;
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_idc[i][k]");
      sli.refLevelIdc(i, k) = (Level::Name) val;

      if (sli.explicitFractionPresentFlag())
      {
        for (int j = 0; j < sli.numSubpics(); j++)
        {
          sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_fraction_minus1[i][j][k]");
          sli.refLevelFraction(i, j, k) = val;
        }
      }
    }
  }

  // update the inference of m_refLevelIdc[][] and m_refLevelFraction[][][]
  if (!sli.hasSublayerInfo)
  {
    sli.fillSublayers();
  }
}

void SEIReader::xParseSEISampleAspectRatioInfo(SEISampleAspectRatioInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag( pDecodedMessageOutputStream,           val,    "sari_cancel_flag" );                      sei.m_sariCancelFlag = val;
  if (!sei.m_sariCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,         val,    "sari_persistence_flag" );                 sei.m_sariPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     8,  val,    "sari_aspect_ratio_idc" );                 sei.m_sariAspectRatioIdc = val;
    if (sei.m_sariAspectRatioIdc == 255)
    {
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_width" );                        sei.m_sariSarWidth = val;
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_height" );                       sei.m_sariSarHeight = val;
    }
  }
}

void SEIReader::xParseSEIExtendedDrapIndication(SEIExtendedDrapIndication& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code( pDecodedMessageOutputStream, 16, val,        "edrap_rap_id_minus1"          );   sei.m_edrapIndicationRapIdMinus1         = val;
  sei_read_flag( pDecodedMessageOutputStream,     val,        "edrap_leading_pictures_decodable_flag" );       sei.m_edrapIndicationLeadingPicturesDecodableFlag = val;
  sei_read_code( pDecodedMessageOutputStream, 12, val,        "edrap_reserved_zero_12bits"          );   sei.m_edrapIndicationReservedZero12Bits = val;
  sei_read_code( pDecodedMessageOutputStream, 3, val,         "edrap_num_ref_rap_pics_minus1"          );   sei.m_edrapIndicationNumRefRapPicsMinus1 = val;
  sei.m_edrapIndicationRefRapId.resize(sei.m_edrapIndicationNumRefRapPicsMinus1 + 1);
  for (int i = 0; i <= sei.m_edrapIndicationNumRefRapPicsMinus1; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 16, val,       "edrap_ref_rap_id[i]"          );
    sei.m_edrapIndicationRefRapId[i] = val;
  }
}

void SEIReader::xParseSEIConstrainedRaslIndication( SEIConstrainedRaslIndication& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
}

void SEIReader::xParseSEINNPostFilterCharacteristics(SEINeuralNetworkPostFilterCharacteristics& sei, uint32_t payloadSize, const SPS* sps, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_code(pDecodedMessageOutputStream, 16, val, "nnpfc_purpose");
  sei.m_purpose = val;
  CHECK(sei.m_purpose >= 64 && sei.m_purpose <= 65535, "Reserved nnpfc_purpose value");

  sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfc_id" );
  sei.m_id = val;
  CHECK((sei.m_id >= 256 && sei.m_id <= 511) || (sei.m_id >= (1<<31) && sei.m_id <= MAX_NNPFC_ID), "Reserved nnpfc_id value, shall ignore the SEI message");

  sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_base_flag");
  sei.m_baseFlag = val;

  sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfc_mode_idc" );
  sei.m_modeIdc = val;

  if (sei.m_modeIdc == POST_FILTER_MODE::URI)
  {
    std::string val2;
    while (!isByteAligned())
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_alignment_zero_bit");
      CHECK(val != 0, "nnpfc_alignment_zero_bit not equal to zero");
    }

    sei_read_string(pDecodedMessageOutputStream, val2, "nnpfc_uri_tag");
    sei.m_uriTag = val2;

    val2 = "";
    sei_read_string(pDecodedMessageOutputStream, val2, "nnpfc_uri");
    sei.m_uri = val2;
  }

  sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_property_present_flag");
  sei.m_propertyPresentFlag = val;

  if (sei.m_propertyPresentFlag)
  {
    ChromaFormat chromaFormatIdc = sps->getChromaFormatIdc();
    uint8_t      subWidthC;
    uint8_t      subHeightC;
    if (chromaFormatIdc == ChromaFormat::_420)
    {
      subWidthC  = 2;
      subHeightC = 2;
    }
    else if (chromaFormatIdc == ChromaFormat::_422)
    {
      subWidthC  = 2;
      subHeightC = 1;
    }
    else
    {
      subWidthC  = 1;
      subHeightC = 1;
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_number_of_input_pictures_minus1");
    sei.m_numberInputDecodedPicturesMinus1 = val;

    sei.m_inputPicOutputFlag.clear();
    sei.m_numInpPicsInOutputTensor = 0;
    if (sei.m_numberInputDecodedPicturesMinus1 > 0)
    {
      bool atLeastOne = false;
      for (int i = 0; i <= sei.m_numberInputDecodedPicturesMinus1; i++)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_input_pic_filtering_flag");
        sei.m_inputPicOutputFlag.push_back((bool)val);
        if (sei.m_inputPicOutputFlag[i])
        {
          atLeastOne = true;
          sei.m_numInpPicsInOutputTensor++;
        }
      }
      if ((sei.m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) == 0)
      {
        CHECK(!atLeastOne, "When picRateUpsamplingFlag is equal to 0 and nnpfc_num_input_pics_minus1 is greater than 0, at least one value of nnpfc_input_pic_filtering_flag[i] shall be greater than 0");
      }
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_absent_input_pic_zero_flag");
      sei.m_absentInputPicZeroFlag = val;
    }
    else
    {
      sei.m_inputPicOutputFlag.push_back(true);
      sei.m_numInpPicsInOutputTensor = 1;
    }

    if((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_out_sub_c_flag");
      sei.m_outSubCFlag = val;

      CHECK(((subWidthC == 2) && (subHeightC == 1) && (sei.m_outSubCFlag == 0)),
            "If SubWidthC is equal to 2 and SubHeightC is equal to 1, nnpfc_out_sub_c_flag shall not be equal to 0");

      if (sei.m_outSubCFlag)
      {
        sei.m_outSubWidthC = 1;
        sei.m_outSubHeightC = 1;
      }
      else
      {
        sei.m_outSubWidthC = 2;
        sei.m_outSubHeightC = 1;
      }
    }

    CHECK(((subWidthC == 1) && (subHeightC == 1)) && ((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0),
          "If SubWidthC is equal to 1 and SubHeightC is equal to 1, nnpfc_purpose & 0x02 shall be equal to 0");

    if((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0), "When chromaUpsamplingFlag is not equal to 0, colourizationFlag shall be equal to 0");

      sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_out_colour_format_idc");
      sei.m_outColourFormatIdc = ChromaFormat(val);
      CHECK(sei.m_outColourFormatIdc == ChromaFormat::_400,
            "The value of nnpfc_out_colour_format_idc shall not be equal to 0");

      sei.m_outSubWidthC  = SPS::getWinUnitX(sei.m_outColourFormatIdc);
      sei.m_outSubHeightC = SPS::getWinUnitY(sei.m_outColourFormatIdc);
    }

    if (((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) == 0) && ((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) == 0))
    {
      sei.m_outSubWidthC  = subWidthC;
      sei.m_outSubHeightC = subHeightC;
    }

    if((sei.m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_width_num_minus1");
      sei.m_picWidthNumeratorMinus1 = val;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_width_denominator_minus1");
      sei.m_picWidthDenominatorMinus1 = val;

      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_height_num_minus1");
      sei.m_picHeightNumeratorMinus1 = val;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_height_denominator_minus1");
      sei.m_picHeightDenominatorMinus1 = val;
      CHECK(sei.m_picWidthNumeratorMinus1 > 65535, "nnpfc_pic_width_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei.m_picWidthDenominatorMinus1 > 65535, "nnpfc_pic_width_denom_minus1 shall be in the range of 0 to 65535");
      CHECK(sei.m_picHeightNumeratorMinus1 > 65535, "nnpfc_pic_height_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei.m_picHeightDenominatorMinus1 > 65535, "nnpfc_pic_height_denom_minus1 shall be in the range of 0 to 65535");
      int scaledHeightRatio = 16 * (sei.m_picHeightNumeratorMinus1 + 1) / (sei.m_picHeightDenominatorMinus1 + 1);
      int scaledWidthRatio = 16 * (sei.m_picWidthNumeratorMinus1 + 1) / (sei.m_picWidthDenominatorMinus1 + 1);

      CHECK((scaledHeightRatio < 1) && (scaledHeightRatio > 256), "The value range of heightRatio shall be in the range of 1/16 to 16, inclusive");
      CHECK((scaledWidthRatio < 1) && (scaledWidthRatio > 256), "The value range of widthRatio shall be in the range of 1/16 to 16, inclusive");
    }

    if((sei.m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) != 0)
    {
      CHECK(sei.m_numberInputDecodedPicturesMinus1 <= 0, "If nnpfc_purpose is FRAME_RATE_UPSAMPLING, nnpfc_num_input_pics_minus1 shall be greater than 0");
      sei.m_numberInterpolatedPictures.resize(sei.m_numberInputDecodedPicturesMinus1);
      bool allZeroFlag = false;
      for (int i = 0; i < sei.m_numberInterpolatedPictures.size(); i++)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_interpolated_pictures");
        sei.m_numberInterpolatedPictures[i] = val;
        if(sei.m_numberInterpolatedPictures[i] > 0)
        {
          allZeroFlag = true;
        }
      }
      CHECK(!allZeroFlag, "At least one value of nnpfc_interpolated_pics[i] shall be greater than 0");
    }


    sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_component_last_flag");
    sei.m_componentLastFlag = val;

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_format_idc");
    sei.m_inpFormatIdc = val;
    CHECK(sei.m_inpFormatIdc > 255, "The value of nnpfc_inp_format_idc shall be in the range of 0 to 255");

    sei_read_uvlc(pDecodedMessageOutputStream,val,"nnpfc_auxiliary_inp_idc");
    sei.m_auxInpIdc = val;
    CHECK(val > 1, "The value of nnpfc_auxiliary_inp_idc shall be in the range of 0 to 1");
    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_order_idc");
    sei.m_inpOrderIdc = val;
    CHECK(val > 3, "The value of nnpfc_inp_order_idc shall be in the range of 0 to 3");
    CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)  && sei.m_inpOrderIdc == 0, "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_inp_order_idc shall not be equal to 0.");

    CHECK((chromaFormatIdc == ChromaFormat::_400) && (sei.m_inpOrderIdc != 0), "When ChromaFormatIdc is equal to 0, nnpfc_inp_order_idc shall be equal to 0");
    CHECK((chromaFormatIdc != ChromaFormat::_420) && (sei.m_inpOrderIdc == 3), "When ChromaFormatIdc is not equal to 1, nnpfc_inp_order_idc shall not be equal to 3");
    CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0) && (sei.m_inpOrderIdc == 0), "When chromaUpsamplingFlag is equal to 1, nnpfc_inp_order_idc shall not be equal to 0");

    if (sei.m_inpFormatIdc == 1)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_tensor_luma_bitdepth_minus8");
        sei.m_inpTensorBitDepthLumaMinus8 = val;
        CHECK(val > 24, "The value of nnpfc_inp_tensor_luma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_tensor_chroma_bitdepth_minus8");
        sei.m_inpTensorBitDepthChromaMinus8 = val;
        CHECK(val > 24, "The value of nnpfc_inp_tensor_chroma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_format_idc");
    sei.m_outFormatIdc = val;
    CHECK(sei.m_outFormatIdc > 255, "The value of nnpfc_out_format_idc shall be in the range of 0 to 255");

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_order_idc");
    sei.m_outOrderIdc = val;
    CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)  && (sei.m_outOrderIdc == 0 || sei.m_outOrderIdc == 3), "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0 or 3.");
    CHECK(((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)  && sei.m_outOrderIdc == 0, "When nnpfc_purpose & 0x20 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0.");

    if (sei.m_outFormatIdc == 1)
    {
      if (sei.m_outOrderIdc != 1)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_tensor_luma_bitdepth_minus8");
        sei.m_outTensorBitDepthLumaMinus8 = val;
        CHECK(val > 24, "The value of nnpfc_out_tensor_luma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
      if (sei.m_outOrderIdc != 0)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_tensor_chroma_bitdepth_minus8");
        sei.m_outTensorBitDepthChromaMinus8 = val; 
        CHECK(val > 24, "The value of nnpfc_out_tensor_chroma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
    }

    if((sei.m_outFormatIdc == 1) && (sei.m_inpFormatIdc == 1) && (sei.m_outOrderIdc > 1) && (sei.m_inpOrderIdc > 1))
    {
      CHECK((sei.m_outTensorBitDepthLumaMinus8 > sei.m_inpTensorBitDepthLumaMinus8) && (sei.m_outTensorBitDepthChromaMinus8 < sei.m_inpTensorBitDepthChromaMinus8), "When outTensorBitDepthLuma is greater than inpTensorBitDepthLuma, outTensorBitDepthChroma shall not be less than inpTensorBitDepthChroma");
      CHECK((sei.m_outTensorBitDepthLumaMinus8 < sei.m_inpTensorBitDepthLumaMinus8) && (sei.m_outTensorBitDepthChromaMinus8 > sei.m_inpTensorBitDepthChromaMinus8), "When outTensorBitDepthChroma is greater than inpTensorBitDepthChroma, outTensorBitDepthLuma shall not be less than inpTensorBitDepthLuma");
    }

    sei_read_flag(pDecodedMessageOutputStream,val,"nnpfc_sep_col_desc_flag");
    sei.m_sepColDescriptionFlag = val;

    if (sei.m_sepColDescriptionFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 8, val,"nnpfc_col_primaries");
      sei.m_colPrimaries = val;
      sei_read_code(pDecodedMessageOutputStream, 8, val,"nnpfc_trans_characteristics");
      sei.m_transCharacteristics = val;
      if (sei.m_outFormatIdc == 1)
      {
        sei_read_code(pDecodedMessageOutputStream, 8, val, "nnpfc_matrix_coeffs");
        sei.m_matrixCoeffs = val;
        CHECK(sei.m_matrixCoeffs == 0 && !(sei.m_outTensorBitDepthChromaMinus8 == sei.m_outTensorBitDepthLumaMinus8 && sei.m_outOrderIdc == 2 && sei.m_outSubHeightC == 1 && sei.m_outSubWidthC == 1),
          "nnpfc_matrix_coeffs shall not be equal to 0 unless the following conditions are true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
        CHECK(sei.m_matrixCoeffs == 8 && !((sei.m_outTensorBitDepthChromaMinus8 == sei.m_outTensorBitDepthLumaMinus8) || (sei.m_outTensorBitDepthChromaMinus8 == (sei.m_outTensorBitDepthLumaMinus8 + 1) && sei.m_outOrderIdc == 2 && sei.m_outSubHeightC == 1 && sei.m_outSubWidthC == 1)),
          "nnpfc_matrix_coeffs shall not be equal to 8 unless one of the following conditions is true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 or "
          "nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 + 1, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
      }
    }

#if JVET_AD0067_INCLUDE_SYNTAX
    if (sei.m_sepColDescriptionFlag & (sei.m_outFormatIdc == 1))
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_full_range_flag");
      sei.m_fullRangeFlag = val;
    }
#endif

    if (sei.m_outOrderIdc != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream,val,"nnpfc_chroma_loc_info_present_flag");
      sei.m_chromaLocInfoPresentFlag = val;
    }
    else
    {
      sei.m_chromaLocInfoPresentFlag = 0;
    }

    CHECK((sei.m_outColourFormatIdc != ChromaFormat::_420) && sei.m_chromaLocInfoPresentFlag, "When nnpfc_out_colour_format_idc is not equal to 1, the value of nnpfc_chroma_loc_info_present_flag shall be equal to 0");
    CHECK((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) && sei.m_chromaLocInfoPresentFlag,"When colourizationFlag is equal to 0, the value of nnpfc_chroma_loc_info_present_flag shall be equal to 0")
    
    if(sei.m_chromaLocInfoPresentFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_chroma_sample_loc_type_frame");
      sei.m_chromaSampleLocTypeFrame = static_cast<Chroma420LocType>(val);
      CHECK(sei.m_chromaSampleLocTypeFrame > Chroma420LocType::UNSPECIFIED, "The value of nnpfc_chroma_sample_loc_type_frame shall be in the range of 0 to 6, inclusive");
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_overlap");
    sei.m_overlap = val;

    sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_constant_patch_size_flag");
    sei.m_constantPatchSizeFlag = val;

    if (sei.m_constantPatchSizeFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_patch_width_minus1");
      sei.m_patchWidthMinus1 = val;

      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_patch_height_minus1");
      sei.m_patchHeightMinus1 = val;
    }
    else
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_extended_patch_width_cd_delta_minus1");
      sei.m_extendedPatchWidthCdDeltaMinus1 = val;

      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_extended_patch_height_cd_delta_minus1");
      sei.m_extendedPatchHeightCdDeltaMinus1 = val;
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_padding_type");
    sei.m_paddingType = val;
  if((sei.m_paddingType >= 5) && (sei.m_paddingType <= 15))
  {
    std::cout<<"Reserved nnpfc_padding_type value, shall ignore the SEI message"<<std::endl;
    return;
  }
  CHECK(sei.m_paddingType > 15, "Values of nnpfc_padding_type greater than 15 shall not be present in bitstreams");

    if (sei.m_paddingType == NNPC_PaddingType::FIXED_PADDING)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_luma_padding_val");
        sei.m_lumaPadding = val;
        CHECK(sei.m_lumaPadding > ((1 << sps->getBitDepth(ChannelType::LUMA)) - 1), "The value of nnpfc_luma_padding_val shall be in the range of 0 to ( 1  <<  BitDepthY ) - 1");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_cb_padding_val");
        sei.m_cbPadding = val;
        CHECK(sei.m_cbPadding > ((1 << sps->getBitDepth(ChannelType::CHROMA)) - 1), "The value of nnpfc_cb_padding_val shall be in the range of 0 to ( 1  <<  BitDepthC ) - 1");

        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_cr_padding_val");
        sei.m_crPadding = val;
        CHECK(sei.m_crPadding > ((1 << sps->getBitDepth(ChannelType::CHROMA)) - 1), "The value of nnpfc_cr_padding_val shall be in the range of 0 to ( 1  <<  BitDepthC ) - 1");
      }
    }

    sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_complexity_info_present_flag");
    sei.m_complexityInfoPresentFlag = val;

    if (sei.m_complexityInfoPresentFlag)
    {
        sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_parameter_type_idc");
        sei.m_parameterTypeIdc = val;
        if (sei.m_parameterTypeIdc != 2)
        {
          sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_log2_parameter_bit_length_minus3");
          sei.m_log2ParameterBitLengthMinus3 = val;
        }

        sei_read_code(pDecodedMessageOutputStream, 6, val, "nnpfc_num_parameters_idc");
        sei.m_numParametersIdc = val;

        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_num_kmac_operations_idc");
        sei.m_numKmacOperationsIdc = val;

        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_total_kilobyte_size");
        sei.m_totalKilobyteSize = val;
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_metadata_extension_num_bits");  // nnpfc_metadata_extension_num_bits shall be equal to 0 in the current edition
    CHECK (val > 2048, "Values of nnpfc_metadata_extension_num_bits greater than 2048 shall not be present in bitstreams");
    for (uint32_t i = 0; i < val; i++)
    {
      uint32_t val2;
      sei_read_code(pDecodedMessageOutputStream, 1, val2, "nnpfc_reserved_metadata_extension"); // Decoders shall ignore the presence and value of nnpfc_reserved_metadata_extension
    }
  }

  if (sei.m_modeIdc == POST_FILTER_MODE::ISO_IEC_15938_17)
  {
    while (!isByteAligned())
    {
      sei_read_flag( pDecodedMessageOutputStream,   val,    "nnpfc_alignment_zero_bit");
      CHECK (val != 0, "nnpfc_alignment_zero_bit not equal to zero");
    }

    int payloadBytesRemaining = getBitstream()->getNumBitsLeft() / 8;
    sei.m_payloadLength = payloadBytesRemaining;
    sei.m_payloadByte = new char[sei.m_payloadLength];
    int code;

    std::string filename = "payloadByte" + std::to_string(sei.m_id) + ".nnr";

    std::ofstream outFile(filename.c_str(), std::ofstream::binary);

    for (int i = 0; i < payloadBytesRemaining; i++)
    {
      sei_read_scode ( pDecodedMessageOutputStream, 8, code, "nnpfc_payload_byte[i]");
      sei.m_payloadByte[i] = (char)code;
      outFile.write((char*)&code, 1);
    }
    outFile.close();
  }
}

void SEIReader::xParseSEINNPostFilterActivation(SEINeuralNetworkPostFilterActivation &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfa_target_id" );
  sei.m_targetId =val;
  CHECK(sei.m_targetId > MAX_NNPFA_ID, "The value of nnpfa_target_id shall be in the range of 0 to 2^32 - 2");
  sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_cancel_flag" );
  sei.m_cancelFlag = val;

  if(!sei.m_cancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_persistence_flag" );
    sei.m_persistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_target_base_flag" );
    sei.m_targetBaseFlag = val;

    sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_no_prev_clvs_flag" );
    sei.m_noPrevCLVSFlag = val;
    if (sei.m_persistenceFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_no_foll_clvs_flag" );
      sei.m_noFollCLVSFlag = val;
    }

    sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfa_num_output_entries" );
    uint32_t numOutputEntries = val;
    sei.m_outputFlag.resize(numOutputEntries);
    for (uint32_t i = 0; i < numOutputEntries; i++)
    {
      sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_output_flag" );
      sei.m_outputFlag[i] = val;
    }
  }
}

void SEIReader::xParseSEIPhaseIndication(SEIPhaseIndication& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_code(pDecodedMessageOutputStream, 8, val, "hor_phase_num");
  sei.m_horPhaseNum = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "hor_phase_den_minus1");
  sei.m_horPhaseDenMinus1 = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "ver_phase_num");
  sei.m_verPhaseNum = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "ver_phase_den_minus1");
  sei.m_verPhaseDenMinus1 = val;

  CHECK(sei.m_horPhaseNum > sei.m_horPhaseDenMinus1 + 1, "The value of hor_phase_num shall be in the range of 0 to hor_phase_den_minus1 + 1, inclusive");
  CHECK(sei.m_verPhaseNum > sei.m_verPhaseDenMinus1 + 1, "The value of ver_phase_num shall be in the range of 0 to ver_phase_den_minus1 + 1, inclusive");
}

#if JVET_S0257_DUMP_360SEI_MESSAGE
void SeiCfgFileDump::write360SeiDump (std::string decoded360MessageFileName, SEIMessages& seis, const SPS* sps)
{
  if (m_360SEIMessageDumped)
  {
    return;
  }

  SEIMessages equirectangularProjectionSEIs = getSeisByType(seis, SEI::PayloadType::EQUIRECTANGULAR_PROJECTION);
  if (!equirectangularProjectionSEIs.empty())
  {
    SEIEquirectangularProjection* sei = (SEIEquirectangularProjection*)equirectangularProjectionSEIs.front();
    xDumpSEIEquirectangularProjection(*sei, sps, decoded360MessageFileName);
    m_360SEIMessageDumped = true;
  }
  else
  {
    SEIMessages generalizedCubemapProjectionSEIs =
      getSeisByType(seis, SEI::PayloadType::GENERALIZED_CUBEMAP_PROJECTION);
    if (!generalizedCubemapProjectionSEIs.empty())
    {
      SEIGeneralizedCubemapProjection* sei = (SEIGeneralizedCubemapProjection*)generalizedCubemapProjectionSEIs.front();
      xDumpSEIGeneralizedCubemapProjection(*sei, sps, decoded360MessageFileName);
      m_360SEIMessageDumped = true;
    }
  }
}

void SeiCfgFileDump::xDumpSEIEquirectangularProjection     (SEIEquirectangularProjection &sei, const SPS* sps, std::string decoded360MessageFileName)
{
  if (!decoded360MessageFileName.empty())
  {
    FILE *fp = fopen(decoded360MessageFileName.c_str(), "w");
    if (fp)
    {
      EnumArray<int, ChromaFormat> chromaFormatTable = { 400, 420, 422, 444 };
      fprintf(fp, "InputBitDepth                 : %d    # Input bitdepth\n", sps->getBitDepth(ChannelType::LUMA));
      fprintf(fp, "InputChromaFormat             : %d    # Ratio of luminance to chrominance samples\n", chromaFormatTable[sps->getChromaFormatIdc()]);
      fprintf(fp, "SourceWidth                   : %d    # Input  frame width\n", sps->getMaxPicWidthInLumaSamples());
      fprintf(fp, "SourceHeight                  : %d    # Input  frame height\n\n", sps->getMaxPicHeightInLumaSamples());

      fprintf(fp, "InputGeometryType             : 0     # 0: equirectangular; 1: cubemap; 2: equalarea; this should be in the cfg of per sequence.\n");
      if (sei.m_erpGuardBandFlag == 1)
      {
        fprintf(fp, "InputPERP                     : 1     # 0: original ERP input; 1: padded ERP input\n");
        fprintf(fp, "CodingPERP                    : 0     # 0: coding with original ERP size; 1: coding with padded ERP\n");
      }
      fclose(fp);
      m_360SEIMessageDumped = true;
    }
    else
    {
      msg( ERROR, "File %s could not be opened.\n", decoded360MessageFileName.c_str() );
    }
  }
}
void SeiCfgFileDump::xDumpSEIGeneralizedCubemapProjection  (SEIGeneralizedCubemapProjection &sei, const SPS* sps, std::string decoded360MessageFileName)
{
  if (!sei.m_gcmpCancelFlag)
  {
    int numFace = sei.m_gcmpPackingType == 4 || sei.m_gcmpPackingType == 5 ? 5 : 6;
    int packingTypeTable[6][2] = {{6, 1}, {3, 2}, {2, 3}, {1, 6}, {1, 5}, {5, 1}};
    int rotationTable[4] = {0, 90, 180, 270};
    std::string packingTypeStr = "";
    std::string gcmpsettingsStr = "";
    std::ostringstream oss;

    packingTypeStr += "SourceFPStructure                 : " + std::to_string(packingTypeTable[sei.m_gcmpPackingType][0]) + " " + std::to_string(packingTypeTable[sei.m_gcmpPackingType][1]);
    gcmpsettingsStr += "InputGCMPSettings                 : ";

    for (int i = 0; i < numFace; i++)
    {
      int rotation = rotationTable[sei.m_gcmpFaceRotation[i]];
      if (sei.m_gcmpFaceIndex[i] == 1)
      {
        rotation = (rotation + 270) % 360 + 360;
      }
      else if (sei.m_gcmpFaceIndex[i] == 2)
      {
        rotation = (rotation + 180) % 360 + 360;
      }
      else
      {
        rotation += 360;
      }
      if (i % packingTypeTable[sei.m_gcmpPackingType][1] == 0)
      {
        packingTypeStr += "   ";
      }
      packingTypeStr += std::to_string(sei.m_gcmpFaceIndex[i]) + " " + std::to_string(rotation) + " ";

      if (sei.m_gcmpMappingFunctionType == 2)
      {
        double a = ((int)sei.m_gcmpFunctionCoeffU[i] + 1) / 128.0;
        double b = ((int)sei.m_gcmpFunctionCoeffV[i] + 1) / 128.0;
        oss.str("");
        oss<<a;
        std::string a_str = oss.str();
        oss.str("");
        oss<<b;
        std::string b_str = oss.str();
        gcmpsettingsStr += a_str + " " + std::to_string(sei.m_gcmpFunctionUAffectedByVFlag[i]) + " " + b_str + " " + std::to_string(sei.m_gcmpFunctionVAffectedByUFlag[i]) + "   ";
      }
    }
    if (!decoded360MessageFileName.empty())
    {
      FILE *fp = fopen(decoded360MessageFileName.c_str(), "w");
      if (fp)
      {
        const EnumArray<int, ChromaFormat> chromaFormatTable = { 400, 420, 422, 444 };
        fprintf(fp, "InputBitDepth                 : %d    # Input bitdepth\n", sps->getBitDepth(ChannelType::LUMA));
        fprintf(fp, "InputChromaFormat             : %d    # Ratio of luminance to chrominance samples\n", chromaFormatTable[sps->getChromaFormatIdc()]);
        fprintf(fp, "SourceWidth                   : %d    # Input  frame width\n", sps->getMaxPicWidthInLumaSamples());
        fprintf(fp, "SourceHeight                  : %d    # Input  frame height\n\n", sps->getMaxPicHeightInLumaSamples());

        fprintf(fp, "InputGeometryType             : 15    # 0: equirectangular; 1: cubemap; 2: equalarea; this should be in the cfg of per sequence.\n");

        packingTypeStr += " # frame packing order: numRows numCols Row0Idx0 ROT Row0Idx1 ROT ... Row1...";
        gcmpsettingsStr += " # mapping function parameters for each face: u coefficient, u affected by v flag, v coefficient, v affected by u flag";
        fprintf(fp, "%s\n", packingTypeStr.c_str());
        fprintf(fp, "InputGCMPMappingType              : %d                                    # 0: CMP; 1: EAC; 2: parameterized CMP\n", (int)sei.m_gcmpMappingFunctionType);
        if ((int)sei.m_gcmpMappingFunctionType == 2)
        {
          fprintf(fp, "%s\n", gcmpsettingsStr.c_str());
        }
        fprintf(fp, "InputGCMPPaddingFlag              : %d                                   # 0: input without guard bands; 1: input with guard bands\n", sei.m_gcmpGuardBandFlag);
        if (sei.m_gcmpGuardBandFlag)
        {
          fprintf(fp, "InputGCMPPaddingType              : %d                                   # 0: unspecified(repetitive padding is used); 1: repetitive padding; 2: copy from neighboring face; 3: geometry padding\n", (int)sei.m_gcmpGuardBandType);
          fprintf(fp, "InputGCMPPaddingExteriorFlag      : %d                                   # 0: guard bands only on discontinuous edges; 1: guard bands on both discontinuous edges and frame boundaries\n", sei.m_gcmpGuardBandBoundaryExteriorFlag);
          fprintf(fp, "InputGCMPPaddingSize              : %d                                   # guard band size for input GCMP\n", (int)sei.m_gcmpGuardBandSamplesMinus1 + 1);
        }
        fclose(fp);
        m_360SEIMessageDumped = true;
      }
      else
      {
        msg( ERROR, "File %s could not be opened.\n", decoded360MessageFileName.c_str() );
      }
    }
  }
}

#endif

void SEIReader::xParseSEISEIManifest(SEIManifest &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  unsigned int val;
  sei_read_code(pDecodedMessageOutputStream, 16, val, "manifest_num_sei_msg_types");
  sei.m_manifestNumSeiMsgTypes = val;
  if (sei.m_manifestNumSeiMsgTypes > 0)
  {
    sei.m_manifestSeiPayloadType.resize(sei.m_manifestNumSeiMsgTypes);
    sei.m_manifestSeiDescription.resize(sei.m_manifestNumSeiMsgTypes);
    for (int i = 0; i < sei.m_manifestNumSeiMsgTypes; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, val, "manifest_sei_payload_types");
      sei.m_manifestSeiPayloadType[i] = static_cast<SEI::PayloadType>(val);
      sei_read_code(pDecodedMessageOutputStream, 8, val, "manifest_sei_description");
      sei.m_manifestSeiDescription[i] = val;
    }
  }
}

void SEIReader::xParseSEISEIPrefixIndication(SEIPrefixIndication &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  unsigned int val;
  unsigned int bitsRead = 0;
  sei_read_code(pDecodedMessageOutputStream, 16, val, "prefix_sei_payload_type");
  sei.m_prefixSeiPayloadType = static_cast<SEI::PayloadType>(val);
  sei_read_code(pDecodedMessageOutputStream, 8, val, "num_sei_prefix_indications_minus1");
  sei.m_numSeiPrefixIndicationsMinus1 = val;
  if (sei.m_numSeiPrefixIndicationsMinus1 >= 0)
  {
    sei.m_numBitsInPrefixIndicationMinus1.resize(sei.m_numSeiPrefixIndicationsMinus1 + 1);
    sei.m_seiPrefixDataBit.resize(sei.m_numSeiPrefixIndicationsMinus1 + 1);
    for (int i = 0; i <= sei.m_numSeiPrefixIndicationsMinus1; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, val, "num_bits_in_prefix_indication_minus1");
      sei.m_numBitsInPrefixIndicationMinus1[i] = val;
      sei.m_seiPrefixDataBit[i].resize(sei.m_numBitsInPrefixIndicationMinus1[i] + 1);
      for (int j = 0; j <= sei.m_numBitsInPrefixIndicationMinus1[i]; j++)
      {
        sei_read_code(pDecodedMessageOutputStream, 1, val, "sei_prefix_data_bit");
        sei.m_seiPrefixDataBit[i][j] = val;
        bitsRead += 1;
      }
      while (bitsRead % 8 != 0)
      {
        sei_read_code(pDecodedMessageOutputStream, 1, val, "byte_alignment_bit_equal_to_one"); 
        CHECK(!val, "error to read/write SEI_prefix_indication::byte_alignment_bit_equal_to_one");
        bitsRead += 1;
      }
    }
  }
}

//! \}
