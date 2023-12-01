/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
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

#include "CommonLib/BitStream.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Slice.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"
#include "SEIwrite.h"

//! \ingroup EncoderLib
//! \{

void SEIWriter::xWriteSEIpayloadData(OutputBitstream &bs, const SEI &sei, HRD &hrd, const uint32_t temporalId,
                                     int SEIPrefixIndicationIdx)
{
  const SEIBufferingPeriod *bp = nullptr;
  switch (sei.payloadType())
  {
  case SEI::PayloadType::USER_DATA_UNREGISTERED:
    xWriteSEIuserDataUnregistered(*static_cast<const SEIuserDataUnregistered*>(&sei));
    break;
  case SEI::PayloadType::DECODING_UNIT_INFO:
    bp = hrd.getBufferingPeriodSEI();
    CHECK (bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Decoding Unit Information SEI");
    xWriteSEIDecodingUnitInfo(*static_cast<const SEIDecodingUnitInfo*>(& sei), *bp, temporalId);
    break;
  case SEI::PayloadType::SCALABLE_NESTING:
    xWriteSEIScalableNesting(bs, *static_cast<const SEIScalableNesting*>(&sei));
    break;
  case SEI::PayloadType::DECODED_PICTURE_HASH:
    xWriteSEIDecodedPictureHash(*static_cast<const SEIDecodedPictureHash*>(&sei));
    break;
  case SEI::PayloadType::BUFFERING_PERIOD:
    xWriteSEIBufferingPeriod(*static_cast<const SEIBufferingPeriod*>(&sei));
    hrd.setBufferingPeriodSEI(static_cast<const SEIBufferingPeriod*>(&sei));
    break;
  case SEI::PayloadType::PICTURE_TIMING:
    bp = hrd.getBufferingPeriodSEI();
    CHECK(bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Picture Timing SEI");
    xWriteSEIPictureTiming(*static_cast<const SEIPictureTiming *>(&sei), *bp, temporalId);
    break;
  case SEI::PayloadType::FRAME_FIELD_INFO:
    xWriteSEIFrameFieldInfo(*static_cast<const SEIFrameFieldInfo *>(&sei));
    break;
  case SEI::PayloadType::DEPENDENT_RAP_INDICATION:
    xWriteSEIDependentRAPIndication(*static_cast<const SEIDependentRAPIndication *>(&sei));
    break;
  case SEI::PayloadType::EXTENDED_DRAP_INDICATION:
    xWriteSEIEdrapIndication(*static_cast<const SEIExtendedDrapIndication *>(&sei));
    break;
  case SEI::PayloadType::FRAME_PACKING:
    xWriteSEIFramePacking(*static_cast<const SEIFramePacking *>(&sei), SEIPrefixIndicationIdx);
    break;
#if GREEN_METADATA_SEI_ENABLED
  case SEI::PayloadType::GREEN_METADATA:
    xWriteSEIGreenMetadataInfo(*static_cast<const SEIGreenMetadataInfo *>(&sei));
    break;
#endif
  case SEI::PayloadType::DISPLAY_ORIENTATION:
    xWriteSEIDisplayOrientation(*static_cast<const SEIDisplayOrientation *>(&sei));
    break;
  case SEI::PayloadType::PARAMETER_SETS_INCLUSION_INDICATION:
    xWriteSEIParameterSetsInclusionIndication(*static_cast<const SEIParameterSetsInclusionIndication *>(&sei));
    break;
  case SEI::PayloadType::MASTERING_DISPLAY_COLOUR_VOLUME:
    xWriteSEIMasteringDisplayColourVolume(*static_cast<const SEIMasteringDisplayColourVolume *>(&sei));
    break;
  case SEI::PayloadType::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
    xWriteSEIAlternativeTransferCharacteristics(*static_cast<const SEIAlternativeTransferCharacteristics *>(&sei));
    break;
  case SEI::PayloadType::EQUIRECTANGULAR_PROJECTION:
    xWriteSEIEquirectangularProjection(*static_cast<const SEIEquirectangularProjection *>(&sei),
                                       SEIPrefixIndicationIdx);
    break;
  case SEI::PayloadType::SPHERE_ROTATION:
    xWriteSEISphereRotation(*static_cast<const SEISphereRotation *>(&sei), SEIPrefixIndicationIdx);
    break;
  case SEI::PayloadType::OMNI_VIEWPORT:
    xWriteSEIOmniViewport(*static_cast<const SEIOmniViewport *>(&sei));
    break;
  case SEI::PayloadType::REGION_WISE_PACKING:
    xWriteSEIRegionWisePacking(*static_cast<const SEIRegionWisePacking *>(&sei), SEIPrefixIndicationIdx);
    break;
  case SEI::PayloadType::GENERALIZED_CUBEMAP_PROJECTION:
    xWriteSEIGeneralizedCubemapProjection(*static_cast<const SEIGeneralizedCubemapProjection *>(&sei),
                                          SEIPrefixIndicationIdx);
    break;
  case SEI::PayloadType::SCALABILITY_DIMENSION_INFO:
    xWriteSEIScalabilityDimensionInfo(*static_cast<const SEIScalabilityDimensionInfo *>(&sei));
    break;
  case SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO:
    xWriteSEIMultiviewAcquisitionInfo(*static_cast<const SEIMultiviewAcquisitionInfo *>(&sei));
    break;
  case SEI::PayloadType::MULTIVIEW_VIEW_POSITION:
    xWriteSEIMultiviewViewPosition(*static_cast<const SEIMultiviewViewPosition *>(&sei));
    break;
  case SEI::PayloadType::ALPHA_CHANNEL_INFO:
    xWriteSEIAlphaChannelInfo(*static_cast<const SEIAlphaChannelInfo *>(&sei));
    break;
  case SEI::PayloadType::DEPTH_REPRESENTATION_INFO:
    xWriteSEIDepthRepresentationInfo(*static_cast<const SEIDepthRepresentationInfo *>(&sei));
    break;
  case SEI::PayloadType::USER_DATA_REGISTERED_ITU_T_T35:
    xWriteSEIUserDataRegistered(*static_cast<const SEIUserDataRegistered *>(&sei));
    break;
  case SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS:
    xWriteSEIFilmGrainCharacteristics(*static_cast<const SEIFilmGrainCharacteristics *>(&sei));
    break;
  case SEI::PayloadType::CONTENT_LIGHT_LEVEL_INFO:
    xWriteSEIContentLightLevelInfo(*static_cast<const SEIContentLightLevelInfo *>(&sei));
    break;
  case SEI::PayloadType::AMBIENT_VIEWING_ENVIRONMENT:
    xWriteSEIAmbientViewingEnvironment(*static_cast<const SEIAmbientViewingEnvironment *>(&sei));
    break;
  case SEI::PayloadType::CONTENT_COLOUR_VOLUME:
    xWriteSEIContentColourVolume(*static_cast<const SEIContentColourVolume *>(&sei));
    break;
  case SEI::PayloadType::COLOUR_TRANSFORM_INFO:
    xWriteSEIColourTransformInfo(*static_cast<const SEIColourTransformInfo *>(&sei));
    break;
  case SEI::PayloadType::SUBPICTURE_LEVEL_INFO:
    xWriteSEISubpictureLevelInfo(*static_cast<const SEISubpicureLevelInfo *>(&sei));
    break;
  case SEI::PayloadType::SAMPLE_ASPECT_RATIO_INFO:
    xWriteSEISampleAspectRatioInfo(*static_cast<const SEISampleAspectRatioInfo *>(&sei));
    break;
  case SEI::PayloadType::PHASE_INDICATION:
    xWriteSEIPhaseIndication(*static_cast<const SEIPhaseIndication *>(&sei));
    break;
  case SEI::PayloadType::ANNOTATED_REGIONS:
    xWriteSEIAnnotatedRegions(*static_cast<const SEIAnnotatedRegions *>(&sei));
    break;
  case SEI::PayloadType::SEI_MANIFEST:
    CHECK((SEIPrefixIndicationIdx), "wrong SEI prefix indication message");
    xWriteSEISEIManifest(*static_cast<const SEIManifest *>(&sei));
    break;
  case SEI::PayloadType::SEI_PREFIX_INDICATION:
    CHECK((SEIPrefixIndicationIdx), "wrong SEI prefix indication message");
    xWriteSEISEIPrefixIndication(bs, *static_cast<const SEIPrefixIndication *>(&sei), hrd, temporalId);
    break;
  case SEI::PayloadType::CONSTRAINED_RASL_ENCODING:
    xWriteSEIConstrainedRaslIndication(*static_cast<const SEIConstrainedRaslIndication *>(&sei));
    break;
  case SEI::PayloadType::SHUTTER_INTERVAL_INFO:
    xWriteSEIShutterInterval(*static_cast<const SEIShutterIntervalInfo *>(&sei));
    break;
  case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS:
    xWriteSEINeuralNetworkPostFilterCharacteristics(
      *static_cast<const SEINeuralNetworkPostFilterCharacteristics *>(&sei));
    break;
  case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_ACTIVATION:
    xWriteSEINeuralNetworkPostFilterActivation(*static_cast<const SEINeuralNetworkPostFilterActivation *>(&sei));
    break;
  case SEI::PayloadType::SEI_PROCESSING_ORDER:
    xWriteSEIProcessingOrder(bs, *static_cast<const SEIProcessingOrderInfo*>(&sei));
    break;
#if JVET_AF0310_PO_NESTING
  case SEI::PayloadType::SEI_PROCESSING_ORDER_NESTING:
    xWriteSEIProcessingOrderNesting(bs, *static_cast<const SEIProcessingOrderNesting*>(&sei));
    break;
#endif
  case SEI::PayloadType::POST_FILTER_HINT:
    xWriteSEIPostFilterHint(*static_cast<const SEIPostFilterHint *>(&sei));
    break;
  default:
    THROW("Trying to write unhandled SEI message");
    break;
  }
  if (SEIPrefixIndicationIdx)
  {
    return;
  }
  xWriteByteAlign();
}

/**
 * marshal all SEI messages in provided list into one bitstream bs
 */
void SEIWriter::writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, HRD &hrd, bool isNested, const uint32_t temporalId)
{
#if ENABLE_TRACING
  if (g_HLSTraceEnable)
    xTraceSEIHeader();
#endif

  OutputBitstream bs_count;

  for (SEIMessages::const_iterator sei=seiList.begin(); sei!=seiList.end(); sei++)
  {
    // calculate how large the payload data is
    // TODO: this would be far nicer if it used vectored buffers
    bs_count.clear();
    setBitstream(&bs_count);

#if ENABLE_TRACING
    bool traceEnable = g_HLSTraceEnable;
    g_HLSTraceEnable = false;
#endif
    xWriteSEIpayloadData(bs_count, **sei, hrd, temporalId);
#if ENABLE_TRACING
    g_HLSTraceEnable = traceEnable;
#endif
    uint32_t payload_data_num_bits = bs_count.getNumberOfWrittenBits();
    CHECK(0 != payload_data_num_bits % 8, "Invalid number of payload data bits");

    setBitstream(&bs);
    uint32_t payloadType = to_underlying((*sei)->payloadType());
    for (; payloadType >= 0xff; payloadType -= 0xff)
    {
      xWriteCode(0xff, 8, "payload_type");
    }
    xWriteCode(payloadType, 8, "payload_type");

    uint32_t payloadSize = payload_data_num_bits/8;
    for (; payloadSize >= 0xff; payloadSize -= 0xff)
    {
      xWriteCode(0xff, 8, "payload_size");
    }
    xWriteCode(payloadSize, 8, "payload_size");

    /* payloadData */
#if ENABLE_TRACING
    if (g_HLSTraceEnable)
    {
      xTraceSEIMessageType((*sei)->payloadType());
    }
#endif

    xWriteSEIpayloadData(bs, **sei, hrd, temporalId);
  }
  if (!isNested)
  {
    xWriteRbspTrailingBits();
  }
}

/**
 * marshal a user_data_unregistered SEI message sei, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIuserDataUnregistered(const SEIuserDataUnregistered &sei)
{
  for (uint32_t i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    xWriteCode(sei.uuid_iso_iec_11578[i], 8 , "uuid_iso_iec_11578[i]");
  }

  for (uint32_t i = 0; i < sei.userDataLength; i++)
  {
    xWriteCode(sei.userData[i], 8 , "user_data_payload_byte");
  }
}

/**
 * marshal a decoded picture hash SEI message, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei)
{
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

  if (traceString != 0) //use of this variable is needed to avoid a compiler error with G++ 4.6.1
  {
    xWriteCode(to_underlying(sei.method), 8, "dph_sei_hash_type");
    xWriteCode(sei.singleCompFlag, 1, "dph_sei_single_component_flag");
    xWriteCode(0, 7, "dph_sei_reserved_zero_7bits");
    for(uint32_t i=0; i<uint32_t(sei.m_pictureHash.hash.size()); i++)
    {
      xWriteCode(sei.m_pictureHash.hash[i], 8, traceString);
    }
  }
}


void SEIWriter::xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SEIBufferingPeriod& bp, const uint32_t temporalId)
{
  xWriteUvlc(sei.m_decodingUnitIdx, "decoding_unit_idx");
  if( !bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    for (int i = temporalId; i <= bp.m_bpMaxSubLayers - 1; i++)
    {
      if (i < bp.m_bpMaxSubLayers - 1)
      {
        xWriteFlag(sei.m_duiSubLayerDelaysPresentFlag[i], "dui_sub_layer_delays_present_flag[i]");
      }
      if( sei.m_duiSubLayerDelaysPresentFlag[i] )
      {
        xWriteCode( sei.m_duSptCpbRemovalDelayIncrement[i], bp.getDuCpbRemovalDelayIncrementLength(), "du_spt_cpb_removal_delay_increment[i]");
      }
    }
  }
  if (!bp.m_decodingUnitDpbDuParamsInPicTimingSeiFlag)
  {
    xWriteFlag(sei.m_dpbOutputDuDelayPresentFlag, "dpb_output_du_delay_present_flag");
  }

  if(sei.m_dpbOutputDuDelayPresentFlag)
  {
    xWriteCode(sei.m_picSptDpbOutputDuDelay, bp.getDpbOutputDelayDuLength(), "pic_spt_dpb_output_du_delay");
  }
}

void SEIWriter::xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei)
{
  xWriteFlag( sei.m_bpNalCpbParamsPresentFlag, "bp_nal_hrd_parameters_present_flag");
  xWriteFlag( sei.m_bpVclCpbParamsPresentFlag, "bp_vcl_hrd_parameters_present_flag");
  CHECK(!sei.m_bpNalCpbParamsPresentFlag && !sei.m_bpVclCpbParamsPresentFlag, "bp_nal_hrd_parameters_present_flag and/or bp_vcl_hrd_parameters_present_flag must be true");
  CHECK (sei.m_initialCpbRemovalDelayLength < 1, "sei.m_initialCpbRemovalDelayLength must be > 0");
  xWriteCode( sei.m_initialCpbRemovalDelayLength - 1, 5, "initial_cpb_removal_delay_length_minus1" );
  CHECK (sei.m_cpbRemovalDelayLength < 1, "sei.m_cpbRemovalDelayLength must be > 0");
  xWriteCode( sei.m_cpbRemovalDelayLength - 1,        5, "cpb_removal_delay_length_minus1" );
  CHECK (sei.m_dpbOutputDelayLength < 1, "sei.m_dpbOutputDelayLength must be > 0");
  xWriteCode( sei.m_dpbOutputDelayLength - 1,         5, "dpb_output_delay_length_minus1" );
  xWriteFlag( sei.m_bpDecodingUnitHrdParamsPresentFlag, "bp_decoding_unit_hrd_params_present_flag"  );
  if( sei.m_bpDecodingUnitHrdParamsPresentFlag )
  {
    CHECK (sei.m_duCpbRemovalDelayIncrementLength < 1, "sei.m_duCpbRemovalDelayIncrementLength must be > 0");
    xWriteCode( sei.m_duCpbRemovalDelayIncrementLength - 1, 5, "du_cpb_removal_delay_increment_length_minus1" );
    CHECK (sei.m_dpbOutputDelayDuLength < 1, "sei.m_dpbOutputDelayDuLength must be > 0");
    xWriteCode( sei.m_dpbOutputDelayDuLength - 1, 5, "dpb_output_delay_du_length_minus1" );
    xWriteFlag( sei.m_decodingUnitCpbParamsInPicTimingSeiFlag, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );
    xWriteFlag(sei.m_decodingUnitDpbDuParamsInPicTimingSeiFlag, "decoding_unit_dpb_du_params_in_pic_timing_sei_flag");
  }

  xWriteFlag( sei.m_concatenationFlag, "concatenation_flag");
  xWriteFlag( sei.m_additionalConcatenationInfoPresentFlag, "additional_concatenation_info_present_flag");
  if (sei.m_additionalConcatenationInfoPresentFlag)
  {
    xWriteCode( sei.m_maxInitialRemovalDelayForConcatenation, sei.m_initialCpbRemovalDelayLength, "max_initial_removal_delay_for_concatenation" );
  }

  CHECK (sei.m_auCpbRemovalDelayDelta < 1, "sei.m_auCpbRemovalDelayDelta must be > 0");
  xWriteCode( sei.m_auCpbRemovalDelayDelta - 1, sei.m_cpbRemovalDelayLength, "au_cpb_removal_delay_delta_minus1" );

  CHECK(sei.m_bpMaxSubLayers < 1, "bp_max_sub_layers_minus1 must be > 0");
  xWriteCode(sei.m_bpMaxSubLayers - 1, 3, "bp_max_sub_layers_minus1");
  if (sei.m_bpMaxSubLayers - 1 > 0)
  {
    xWriteFlag(sei.m_cpbRemovalDelayDeltasPresentFlag, "cpb_removal_delay_deltas_present_flag");
  }

  if (sei.m_cpbRemovalDelayDeltasPresentFlag)
  {
    CHECK (sei.m_numCpbRemovalDelayDeltas < 1, "m_numCpbRemovalDelayDeltas must be > 0");
    xWriteUvlc( sei.m_numCpbRemovalDelayDeltas - 1, "num_cpb_removal_delay_deltas_minus1" );
    for( int i = 0; i < sei.m_numCpbRemovalDelayDeltas; i ++ )
    {
      xWriteCode( sei.m_cpbRemovalDelayDelta[i],        sei.m_cpbRemovalDelayLength, "cpb_removal_delay_delta[i]" );
    }
  }
  CHECK (sei.m_bpCpbCnt < 1, "sei.m_bpCpbCnt must be > 0");
  xWriteUvlc( sei.m_bpCpbCnt - 1, "bp_cpb_cnt_minus1");
  if (sei.m_bpMaxSubLayers - 1 > 0)
  {
    xWriteFlag(sei.m_sublayerInitialCpbRemovalDelayPresentFlag, "bp_sublayer_initial_cpb_removal_delay_present_flag");
  }
  for (int i = (sei.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : sei.m_bpMaxSubLayers - 1); i < sei.m_bpMaxSubLayers; i++)
  {
    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei.m_bpNalCpbParamsPresentFlag ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei.m_bpVclCpbParamsPresentFlag ) ) )
      {
        for( int j = 0; j < sei.m_bpCpbCnt; j ++ )
        {
          xWriteCode( sei.m_initialCpbRemovalDelay[i][j][nalOrVcl],  sei.m_initialCpbRemovalDelayLength,           "initial_cpb_removal_delay[i][j][nalOrVcl]" );
          xWriteCode( sei.m_initialCpbRemovalOffset[i][j][nalOrVcl], sei.m_initialCpbRemovalDelayLength,           "initial_cpb_removal_delay_offset[i][j][nalOrVcl]" );
        }
      }
    }
  }
  if (sei.m_bpMaxSubLayers-1 > 0)
  {
    xWriteFlag(sei.m_sublayerDpbOutputOffsetsPresentFlag, "bp_sublayer_dpb_output_offsets_present_flag");
  }

  if(sei.m_sublayerDpbOutputOffsetsPresentFlag)
  {
    for(int i = 0; i < sei.m_bpMaxSubLayers - 1; i++)
    {
      xWriteUvlc( sei.m_dpbOutputTidOffset[i], "dpb_output_tid_offset[i]" );
    }
  }
  xWriteFlag(sei.m_altCpbParamsPresentFlag, "bp_alt_cpb_params_present_flag");
  if (sei.m_altCpbParamsPresentFlag)
  {
    xWriteFlag(sei.m_useAltCpbParamsFlag, "use_alt_cpb_params_flag");
  }

}

void SEIWriter::xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SEIBufferingPeriod &bp, const uint32_t temporalId)
{

  xWriteCode( sei.m_auCpbRemovalDelay[bp.m_bpMaxSubLayers - 1] - 1, bp.m_cpbRemovalDelayLength,               "pt_cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  for (int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i++)
  {
    xWriteFlag(sei.m_ptSubLayerDelaysPresentFlag[i], "pt_sublayer_delays_present_flag[i]");
    if (sei.m_ptSubLayerDelaysPresentFlag[i])
    {
      if (bp.m_cpbRemovalDelayDeltasPresentFlag)
      {
        xWriteFlag(sei.m_cpbRemovalDelayDeltaEnabledFlag[i], "pt_cpb_removal_delay_delta_enabled_flag[i]");
      }
      if (sei.m_cpbRemovalDelayDeltaEnabledFlag[i])
      {
        if ((bp.m_numCpbRemovalDelayDeltas - 1) > 0)
        {
          xWriteCode(sei.m_cpbRemovalDelayDeltaIdx[i], ceilLog2(bp.m_numCpbRemovalDelayDeltas), "pt_cpb_removal_delay_delta_idx[i]");
        }
      }
      else
      {
        xWriteCode(sei.m_auCpbRemovalDelay[i] - 1, bp.m_cpbRemovalDelayLength, "pt_cpb_removal_delay_minus1[i]");
      }
    }
  }
  xWriteCode(sei.m_picDpbOutputDelay, bp.m_dpbOutputDelayLength, "pt_dpb_output_delay");
  if( bp.m_altCpbParamsPresentFlag )
  {
    xWriteFlag( sei.m_cpbAltTimingInfoPresentFlag, "cpb_alt_timing_info_present_flag" );
    if( sei.m_cpbAltTimingInfoPresentFlag )
    {
      if (bp.m_bpNalCpbParamsPresentFlag)
      {
        for (int i = (bp.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.m_bpMaxSubLayers - 1); i <= bp.m_bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.m_bpCpbCnt; j++)
          {
            xWriteCode(sei.m_nalCpbAltInitialRemovalDelayDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "nal_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            xWriteCode(sei.m_nalCpbAltInitialRemovalOffsetDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "nal_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
          }
          xWriteCode(sei.m_nalCpbDelayOffset[i], bp.m_cpbRemovalDelayLength, "nal_cpb_delay_offset[ i ]");
          xWriteCode(sei.m_nalDpbDelayOffset[i], bp.m_dpbOutputDelayLength, "nal_dpb_delay_offset[ i ]");
        }
      }

      if (bp.m_bpVclCpbParamsPresentFlag)
      {
        for (int i = (bp.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.m_bpMaxSubLayers - 1);
             i <= bp.m_bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.m_bpCpbCnt; j++)
          {
            xWriteCode(sei.m_vclCpbAltInitialRemovalDelayDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "vcl_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            xWriteCode(sei.m_vclCpbAltInitialRemovalOffsetDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "vcl_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
          }
          xWriteCode(sei.m_vclCpbDelayOffset[i], bp.m_cpbRemovalDelayLength, "vcl_cpb_delay_offset[ i ]");
          xWriteCode(sei.m_vclDpbDelayOffset[i], bp.m_dpbOutputDelayLength,  "vcl_dpb_delay_offset[ i ]");
        }
      }
    }
  }
  if (bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitDpbDuParamsInPicTimingSeiFlag)

  {
    xWriteCode( sei.m_picDpbOutputDuDelay, bp.m_dpbOutputDelayDuLength, "pic_dpb_output_du_delay" );
  }
  if( bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    xWriteUvlc( sei.m_numDecodingUnitsMinus1, "num_decoding_units_minus1" );
    if (sei.m_numDecodingUnitsMinus1 > 0)
    {
      xWriteFlag( sei.m_duCommonCpbRemovalDelayFlag, "du_commmon_cpb_removal_delay_flag" );
      if( sei.m_duCommonCpbRemovalDelayFlag )
      {
        for( int i = temporalId; i <= bp.m_bpMaxSubLayers - 1; i ++ )
        {
          if( sei.m_ptSubLayerDelaysPresentFlag[i] )
          {
            xWriteCode( sei.m_duCommonCpbRemovalDelayMinus1[i], bp.m_duCpbRemovalDelayIncrementLength, "du_common_cpb_removal_delay_increment_minus1[i]" );
          }
        }
      }
      for( int i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
      {
        xWriteUvlc( sei.m_numNalusInDuMinus1[i], "num_nalus_in_du_minus1[i]" );
        if( !sei.m_duCommonCpbRemovalDelayFlag && i < sei.m_numDecodingUnitsMinus1 )
        {
          for( int j = temporalId; j <= bp.m_bpMaxSubLayers - 1; j ++ )
          {
            if( sei.m_ptSubLayerDelaysPresentFlag[j] )
            {
              xWriteCode( sei.m_duCpbRemovalDelayMinus1[i * bp.m_bpMaxSubLayers + j], bp.m_duCpbRemovalDelayIncrementLength, "du_cpb_removal_delay_increment_minus1[i][j]" );
            }
          }
        }
      }
    }
  }
  xWriteCode( sei.m_ptDisplayElementalPeriodsMinus1, 8,       "pt_display_elemental_periods_minus1" );
}

void SEIWriter::xWriteSEIFrameFieldInfo(const SEIFrameFieldInfo& sei)
{
  xWriteFlag( sei.m_fieldPicFlag ? 1 : 0,                    "ffi_field_pic_flag" );
  if (sei.m_fieldPicFlag)
  {
    xWriteFlag( sei.m_bottomFieldFlag ? 1 : 0,               "ffi_bottom_field_flag" );
    xWriteFlag( sei.m_pairingIndicatedFlag ? 1 : 0,          "ffi_pairing_indicated_flag" );
    if (sei.m_pairingIndicatedFlag)
    {
      xWriteFlag( sei.m_pairedWithNextFieldFlag ? 1 : 0,     "ffi_paired_with_next_field_flag" );
    }
  }
  else
  {
    xWriteFlag( sei.m_displayFieldsFromFrameFlag ? 1 : 0,     "ffi_display_fields_from_frame_flag" );
    if (sei.m_displayFieldsFromFrameFlag)
    {
      xWriteFlag( sei.m_topFieldFirstFlag ? 1 : 0,            "ffi_top_field_first_flag" );
    }
    xWriteCode( sei.m_displayElementalPeriodsMinus1, 8,       "ffi_display_elemental_periods_minus1" );
  }
  xWriteCode( sei.m_sourceScanType, 2,                        "ffi_source_scan_type" );
  xWriteFlag( sei.m_duplicateFlag ? 1 : 0,                    "ffi_duplicate_flag" );
}

void SEIWriter::xWriteSEIDependentRAPIndication(const SEIDependentRAPIndication& /*sei*/)
{
  // intentionally empty
}

void SEIWriter::xWriteSEIEdrapIndication(const SEIExtendedDrapIndication& sei)
{
  xWriteCode( sei.m_edrapIndicationRapIdMinus1, 16,        "edrap_rap_id_minsu1" );
  xWriteFlag( sei.m_edrapIndicationLeadingPicturesDecodableFlag ? 1 : 0, "edrap_leading_pictures_decodable_flag" );
  xWriteCode( sei.m_edrapIndicationReservedZero12Bits, 12, "edrap_reserved_zero_12bits" );
  xWriteCode( sei.m_edrapIndicationNumRefRapPicsMinus1, 3, "edrap_num_ref_rap_pics_minus1" );
  for (int i = 0; i <= sei.m_edrapIndicationNumRefRapPicsMinus1; i++)
  {
    xWriteCode( sei.m_edrapIndicationRefRapId[i], 16, "edrap_ref_rap_id[i]" );
  }
}

void SEIWriter::xWriteSEIScalableNesting(OutputBitstream& bs, const SEIScalableNesting& sei)
{
  CHECK (sei.m_nestedSEIs.size()<1, "There must be at lease one SEI message nested in the scalable nesting SEI.")

  xWriteFlag(sei.m_snOlsFlag, "sn_ols_flag");
  xWriteFlag(sei.m_snSubpicFlag, "sn_subpic_flag");
  if (sei.m_snOlsFlag)
  {
    xWriteUvlc(sei.m_snNumOlssMinus1, "sn_num_olss_minus1");
    for (uint32_t i = 0; i <= sei.m_snNumOlssMinus1; i++)
    {
      xWriteUvlc(sei.m_snOlsIdxDeltaMinus1[i], "sn_ols_idx_delta_minus1[i]");
    }
  }
  else
  {
    xWriteFlag(sei.m_snAllLayersFlag, "sn_all_layers_flag");
    if (!sei.m_snAllLayersFlag)
    {
      xWriteUvlc(sei.m_snNumLayersMinus1, "sn_num_layers");
      for (uint32_t i = 1; i <= sei.m_snNumLayersMinus1; i++)
      {
        xWriteCode(sei.m_snLayerId[i], 6, "sn_layer_id");
      }
    }
  }
  if (sei.m_snSubpicFlag)
  {
    xWriteUvlc( sei.m_snNumSubpics - 1, "sn_num_subpics_minus1");
    CHECK(sei.m_snSubpicIdLen < 1, "sn_subpic_id_len_minus1 must be >= 0");
    xWriteUvlc( sei.m_snSubpicIdLen - 1, "sn_subpic_id_len_minus1");
    for (uint32_t i = 0; i < sei.m_snNumSubpics; i++)
    {
      xWriteCode(sei.m_snSubpicId[i], sei.m_snSubpicIdLen, "sn_subpic_id[i]");
    }
  }

  xWriteUvlc( (uint32_t)sei.m_nestedSEIs.size() - 1, "sn_num_seis_minus1");

  // byte alignment
  while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    xWriteFlag(0, "sn_zero_bit");
  }

  SEIMessages bufferingPeriod = getSeisByType(sei.m_nestedSEIs, SEI::PayloadType::BUFFERING_PERIOD);
  if (!bufferingPeriod.empty())
  {
    SEIBufferingPeriod *bp = (SEIBufferingPeriod*)bufferingPeriod.front();
    m_nestingHrd.setBufferingPeriodSEI(bp);
  }

  // write nested SEI messages
  writeSEImessages(bs, sei.m_nestedSEIs, m_nestingHrd, true, 0);
}

void SEIWriter::xWriteSEIFramePacking(const SEIFramePacking &sei, int SEIPrefixIndicationIdx)
{
  if (SEIPrefixIndicationIdx)
  {
    int numBits = 0;
    numBits += 2 * floorLog2(sei.m_arrangementId + 1) + 1;
    if (!sei.m_arrangementCancelFlag) 
    {
      numBits += 9;
    }
    else
    {
      numBits += 2;
    }
    xWriteCode(numBits - 1, 16, "num_bits_in_prefix_indication_minus1");
  }

  xWriteUvlc( sei.m_arrangementId,                  "fp_arrangement_id" );
  xWriteFlag( sei.m_arrangementCancelFlag,          "fp_arrangement_cancel_flag" );

  if( sei.m_arrangementCancelFlag == 0 )
  {
    xWriteCode( sei.m_arrangementType, 7,           "fp_arrangement_type" );
    if (SEIPrefixIndicationIdx)
    {
      return;
    }
    xWriteFlag( sei.m_quincunxSamplingFlag,         "fp_quincunx_sampling_flag" );
    xWriteCode( sei.m_contentInterpretationType, 6, "fp_content_interpretation_type" );
    xWriteFlag( sei.m_spatialFlippingFlag,          "fp_spatial_flipping_flag" );
    xWriteFlag( sei.m_frame0FlippedFlag,            "fp_frame0_flipped_flag" );
    xWriteFlag( sei.m_fieldViewsFlag,               "fp_field_views_flag" );
    xWriteFlag( sei.m_currentFrameIsFrame0Flag,     "fp_current_frame_is_frame0_flag" );

    xWriteFlag( sei.m_frame0SelfContainedFlag,      "fp_frame0_self_contained_flag" );
    xWriteFlag( sei.m_frame1SelfContainedFlag,      "fp_frame1_self_contained_flag" );

    if(sei.m_quincunxSamplingFlag == 0 && sei.m_arrangementType != 5)
    {
      xWriteCode( sei.m_frame0GridPositionX, 4,     "fp_frame0_grid_position_x" );
      xWriteCode( sei.m_frame0GridPositionY, 4,     "fp_frame0_grid_position_y" );
      xWriteCode( sei.m_frame1GridPositionX, 4,     "fp_frame1_grid_position_x" );
      xWriteCode( sei.m_frame1GridPositionY, 4,     "fp_frame1_grid_position_y" );
    }

    xWriteCode( sei.m_arrangementReservedByte, 8,   "fp_arrangement_reserved_byte" );
    xWriteFlag( sei.m_arrangementPersistenceFlag,   "fp_arrangement_persistence_flag" );
  }

  xWriteFlag( sei.m_upsampledAspectRatio,           "fp_upsampled_aspect_ratio_flag" );
}


void SEIWriter::xWriteSEIDisplayOrientation(const SEIDisplayOrientation& sei)
{
  xWriteFlag(sei.m_doCancelFlag, "display_orientation_cancel_flag");

  if (sei.m_doCancelFlag == 0)
  {
    xWriteFlag(sei.m_doPersistenceFlag, "display_orientation_persistence_flag");
    xWriteCode(sei.m_doTransformType, 3, "display_orientation_transform_type");
    xWriteCode(0, 3, "display_orientation_reserved_zero_3bits");
  }
}

void SEIWriter::xWriteSEIParameterSetsInclusionIndication(const SEIParameterSetsInclusionIndication& sei)
{
  xWriteFlag(sei.m_selfContainedClvsFlag, "psii_self_contained_clvs_flag");
}

void SEIWriter::xWriteSEIMasteringDisplayColourVolume(const SEIMasteringDisplayColourVolume& sei)
{
  xWriteCode( sei.values.primaries[0][0],  16,  "mdcv_display_primaries_x[0]" );
  xWriteCode( sei.values.primaries[0][1],  16,  "mdcv_display_primaries_y[0]" );

  xWriteCode( sei.values.primaries[1][0],  16,  "mdcv_display_primaries_x[1]" );
  xWriteCode( sei.values.primaries[1][1],  16,  "mdcv_display_primaries_y[1]" );

  xWriteCode( sei.values.primaries[2][0],  16,  "mdcv_display_primaries_x[2]" );
  xWriteCode( sei.values.primaries[2][1],  16,  "mdcv_display_primaries_y[2]" );

  xWriteCode( sei.values.whitePoint[0],    16,  "mdcv_white_point_x" );
  xWriteCode( sei.values.whitePoint[1],    16,  "mdcv_white_point_y" );

  xWriteCode( sei.values.maxLuminance,     32,  "mdcv_max_display_mastering_luminance" );
  xWriteCode( sei.values.minLuminance,     32,  "mdcv_min_display_mastering_luminance" );
}

void SEIWriter::xWriteSEISEIManifest(const SEIManifest &sei)
{
  xWriteCode(sei.m_manifestNumSeiMsgTypes, 16, "manifest_num_sei_msg_types");
  for (int i = 0; i < sei.m_manifestNumSeiMsgTypes; i++)
  {
    xWriteCode(to_underlying(sei.m_manifestSeiPayloadType[i]), 16, "manifest_sei_payload_types");
    xWriteCode(sei.m_manifestSeiDescription[i], 8, "manifest_sei_description");
  }
}

//SEI prefix indication
void SEIWriter::xWriteSEISEIPrefixIndication(OutputBitstream &bs, const SEIPrefixIndication &sei, HRD &hrd, const uint32_t temporalId)
{
  xWriteCode(to_underlying(sei.m_prefixSeiPayloadType), 16, "prefix_sei_payload_type");
  int idx = sei.m_numSeiPrefixIndicationsMinus1 + 1;
  //If num_sei_prefix_indication cannot be determined during initialization, then determine when writing prefix databits
  if (idx <= 1) 
  {
    xWriteCode(sei.m_numSeiPrefixIndicationsMinus1, 8, "num_sei_prefix_indications_minus1"); 
  }
  // By writing SEI prefix indication recursively, you only need to pass in SEIPrefixIndicationIdx in the corresponding
  // function and add the SEI prefix syntax elements. At present, only part of SEI can be written in SEI prefix
  // indication. If it needs to be added later, the corresponding databit should be determined
  xWriteSEIpayloadData(bs, *static_cast<const SEI *>(sei.m_payload), hrd, temporalId, idx);
  xWriteSEIPrefixIndicationByteAlign();
}

void SEIWriter::xWriteSEIPrefixIndicationByteAlign() {
  while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    xWriteFlag(1, "byte_alignment_bit_equal_to_one");
  } 
}
// ~SEI prefix indication

void SEIWriter::xWriteSEIAnnotatedRegions(const SEIAnnotatedRegions &sei)
{
  xWriteFlag(sei.m_hdr.m_cancelFlag, "ar_cancel_flag");
  if (!sei.m_hdr.m_cancelFlag)
  {
    xWriteFlag(sei.m_hdr.m_notOptimizedForViewingFlag, "ar_not_optimized_for_viewing_flag");
    xWriteFlag(sei.m_hdr.m_trueMotionFlag, "ar_true_motion_flag");
    xWriteFlag(sei.m_hdr.m_occludedObjectFlag, "ar_occluded_object_flag");
    xWriteFlag(sei.m_hdr.m_partialObjectFlagPresentFlag, "ar_partial_object_flag_present_flag");
    xWriteFlag(sei.m_hdr.m_objectLabelPresentFlag, "ar_object_label_present_flag");
    xWriteFlag(sei.m_hdr.m_objectConfidenceInfoPresentFlag, "ar_object_confidence_info_present_flag");
    if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
    {
      assert(sei.m_hdr.m_objectConfidenceLength <= 16 && sei.m_hdr.m_objectConfidenceLength>0);
      xWriteCode((sei.m_hdr.m_objectConfidenceLength - 1), 4, "ar_object_confidence_length_minus_1");
    }
    if (sei.m_hdr.m_objectLabelPresentFlag)
    {
      xWriteFlag(sei.m_hdr.m_objectLabelLanguagePresentFlag, "ar_object_label_language_present_flag");
      if (sei.m_hdr.m_objectLabelLanguagePresentFlag)
      {
        xWriteByteAlign();
        assert(sei.m_hdr.m_annotatedRegionsObjectLabelLang.size()<256);
        for (uint32_t j = 0; j < sei.m_hdr.m_annotatedRegionsObjectLabelLang.size(); j++)
        {
          char ch = sei.m_hdr.m_annotatedRegionsObjectLabelLang[j];
          xWriteCode(ch, 8, "ar_object_label_language");
        }
        xWriteCode('\0', 8, "ar_label_language");
      }
      xWriteUvlc((uint32_t)sei.m_annotatedLabels.size(), "ar_num_label_updates");
      assert(sei.m_annotatedLabels.size()<256);
      for(auto it=sei.m_annotatedLabels.begin(); it!=sei.m_annotatedLabels.end(); it++)
      {
        assert(it->first < 256);
        xWriteUvlc(it->first, "ar_label_idx[]");
        const SEIAnnotatedRegions::AnnotatedRegionLabel &ar=it->second;
        xWriteFlag(!ar.labelValid, "ar_label_cancel_flag");
        if (ar.labelValid)
        {
          xWriteByteAlign();
          assert(ar.label.size()<256);
          for (uint32_t j = 0; j < ar.label.size(); j++)
          {
            char ch = ar.label[j];
            xWriteCode(ch, 8, "ar_label[]");
          }
          xWriteCode('\0', 8, "ar_label[]");
        }
      }
    }

    xWriteUvlc((uint32_t)sei.m_annotatedRegions.size(), "ar_num_object_updates");
    assert(sei.m_annotatedRegions.size()<256);
    for (auto it=sei.m_annotatedRegions.begin(); it!=sei.m_annotatedRegions.end(); it++)
    {
      const SEIAnnotatedRegions::AnnotatedRegionObject &ar = it->second;
      xWriteUvlc(it->first, "ar_object_idx");
      xWriteFlag(ar.objectCancelFlag, "ar_object_cancel_flag");
      if (!ar.objectCancelFlag)
      {
        if (sei.m_hdr.m_objectLabelPresentFlag)
        {
          xWriteFlag(ar.objectLabelValid, "ar_object_label_update_flag");
          if (ar.objectLabelValid)
          {
            assert(ar.objLabelIdx<256);
            xWriteUvlc(ar.objLabelIdx, "ar_object_label_idx");
          }
        }
        xWriteFlag(ar.boundingBoxValid, "ar_object_bounding_box_update_flag");
        if (ar.boundingBoxValid)
        {
          xWriteFlag(ar.boundingBoxCancelFlag, "ar_bounding_box_cancel_flag");
          if (!ar.boundingBoxCancelFlag)
          {
            xWriteCode(ar.boundingBoxTop,   16, "ar_bounding_box_top");
            xWriteCode(ar.boundingBoxLeft,  16, "ar_bounding_box_left");
            xWriteCode(ar.boundingBoxWidth, 16, "ar_bounding_box_width");
            xWriteCode(ar.boundingBoxHeight,16, "ar_bounding_box_height");
            if (sei.m_hdr.m_partialObjectFlagPresentFlag)
            {
              xWriteUvlc(ar.partialObjectFlag, "ar_partial_object_flag");
            }
            if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
            {
              assert(ar.objectConfidence < (1<<sei.m_hdr.m_objectConfidenceLength));
              xWriteCode(ar.objectConfidence, sei.m_hdr.m_objectConfidenceLength, "ar_object_confidence");
            }
          }
        }
      }
    }
  }
}
void SEIWriter::xWriteByteAlign()
{
  if( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    xWriteFlag( 1, "payload_bit_equal_to_one" );
    while( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0 )
    {
      xWriteFlag( 0, "payload_bit_equal_to_zero" );
    }
  }
}

void SEIWriter::xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei)
{
  xWriteCode(sei.m_preferredTransferCharacteristics, 8, "preferred_transfer_characteristics");
}

void SEIWriter::xWriteSEIEquirectangularProjection(const SEIEquirectangularProjection &sei, int SEIPrefixIndicationIdx)
{
  if (SEIPrefixIndicationIdx)
  {
    int numBits = 5;
    if (sei.m_erpGuardBandFlag) 
    {
      numBits += 19;
    }   
    xWriteCode(numBits - 1, 16, "num_bits_in_prefix_indication_minus1");
  }

  xWriteFlag( sei.m_erpCancelFlag, "erp_cancel_flag" );
  if( !sei.m_erpCancelFlag )
  {
    xWriteFlag( sei.m_erpPersistenceFlag, "erp_persistence_flag" );
    xWriteFlag( sei.m_erpGuardBandFlag,   "erp_guard_band_flag" );
    xWriteCode( 0, 2, "erp_reserved_zero_2bits" );
    if ( sei.m_erpGuardBandFlag == 1)
    {
      xWriteCode( sei.m_erpGuardBandType,       3, "erp_guard_band_type" );
      xWriteCode( sei.m_erpLeftGuardBandWidth,  8, "erp_left_guard_band_width" );
      xWriteCode( sei.m_erpRightGuardBandWidth, 8, "erp_right_guard_band_width" );
    }
  }
}

void SEIWriter::xWriteSEISphereRotation(const SEISphereRotation &sei, int SEIPrefixIndicationIdx)
{
  if (SEIPrefixIndicationIdx)
  {
    if (sei.m_sphereRotationCancelFlag)
    {
      xWriteCode(0,                                 8, "num_sei_prefix_indications_minus1");
    }
    else
    {
      xWriteCode(1,                                 8, "num_sei_prefix_indications_minus1");
    }
    int numBits = 8;
    xWriteCode(numBits - 1,                        16, "num_bits_in_prefix_indication_minus1");
  }
  xWriteFlag( sei.m_sphereRotationCancelFlag,           "sphere_rotation_cancel_flag" );
  if( !sei.m_sphereRotationCancelFlag )
  {
    xWriteFlag( sei.m_sphereRotationPersistenceFlag,    "sphere_rotation_persistence_flag" );
    xWriteCode( 0,                                   6, "sphere_rotation_reserved_zero_6bits" );
    if (SEIPrefixIndicationIdx >= 2)
    {
      xWriteSEIPrefixIndicationByteAlign();
      int numBits2 = 8 + 32 + 32 + 32;
      xWriteCode(numBits2 - 1,                      16, "num_bits_in_prefix_indication_minus1");
      xWriteFlag(sei.m_sphereRotationCancelFlag, "sphere_rotation_cancel_flag");
      xWriteFlag(sei.m_sphereRotationPersistenceFlag, "sphere_rotation_persistence_flag");
      xWriteCode(0, 6, "sphere_rotation_reserved_zero_6bits");
    }
    xWriteSCode(sei.m_sphereRotationYaw,            32, "sphere_rotation_yaw" );
    xWriteSCode(sei.m_sphereRotationPitch,          32, "sphere_rotation_pitch" );
    xWriteSCode(sei.m_sphereRotationRoll,           32, "sphere_rotation_roll" );
  }
}

void SEIWriter::xWriteSEIOmniViewport(const SEIOmniViewport &sei)
{
  xWriteCode( sei.m_omniViewportId,     10,        "omni_viewport_id" );
  xWriteFlag( sei.m_omniViewportCancelFlag,        "omni_viewport_cancel_flag" );
  if ( !sei.m_omniViewportCancelFlag )
  {
    xWriteFlag( sei.m_omniViewportPersistenceFlag, "omni_viewport_persistence_flag" );
    const uint32_t numRegions = (uint32_t) sei.m_omniViewportRegions.size();
    xWriteCode( numRegions - 1, 4,                 "omni_viewport_cnt_minus1" );
    for(uint32_t region=0; region<numRegions; region++)
    {
      const SEIOmniViewport::OmniViewport &viewport=sei.m_omniViewportRegions[region];
      xWriteSCode( viewport.azimuthCentre,     32, "omni_viewport_azimuth_centre"   );
      xWriteSCode( viewport.elevationCentre,   32, "omni_viewport_elevation_centre" );
      xWriteSCode( viewport.tiltCentre,        32, "omni_viewport_tilt_center" );
      xWriteCode( viewport.horRange,           32, "omni_viewport_hor_range[i]" );
      xWriteCode( viewport.verRange,           32, "omni_viewport_ver_range[i]" );
    }
  }
}

void SEIWriter::xWriteSEIRegionWisePacking(const SEIRegionWisePacking &sei, int SEIPrefixIndicationIdx)
{
  if (SEIPrefixIndicationIdx)
  {
    xWriteCode(0, 8, "num_sei_prefix_indications_minus1");
    int numBits = 1;
    if (!sei.m_rwpCancelFlag)
    {
      numBits += 111;
    }
    xWriteCode(numBits - 1,                                       16,       "num_bits_in_prefix_indication_minus1");
  }
  xWriteFlag( sei.m_rwpCancelFlag,                                           "rwp_cancel_flag" );
  if(!sei.m_rwpCancelFlag)
  {
    xWriteFlag( sei.m_rwpPersistenceFlag,                                    "rwp_persistence_flag" );
    xWriteFlag( sei.m_constituentPictureMatchingFlag,                        "rwp_constituent_picture_matching_flag" );
    xWriteCode( 0, 5,                                                        "rwp_reserved_zero_5bits" );
    xWriteCode( (uint32_t)sei.m_numPackedRegions,                 8,         "rwp_num_packed_regions" );
    xWriteCode( (uint32_t)sei.m_projPictureWidth,                 32,        "rwp_proj_picture_width" );
    xWriteCode( (uint32_t)sei.m_projPictureHeight,                32,        "rwp_proj_picture_height" );
    xWriteCode( (uint32_t)sei.m_packedPictureWidth,               16,        "rwp_packed_picture_width" );
    xWriteCode( (uint32_t)sei.m_packedPictureHeight,              16,        "rwp_packed_picture_height" );
    if (SEIPrefixIndicationIdx)
    {
      // don't write full message
      return;
    }
    for( int i=0; i < sei.m_numPackedRegions; i++ )
    {
      xWriteCode( 0, 4,                                                      "rwp_reserved_zero_4bits" );
      xWriteCode( (uint32_t)sei.m_rwpTransformType[i],            3,         "rwp_transform_type" );
      xWriteFlag( sei.m_rwpGuardBandFlag[i],                                 "rwp_guard_band_flag" );
      xWriteCode( (uint32_t)sei.m_projRegionWidth[i],             32,        "rwp_proj_region_width" );
      xWriteCode( (uint32_t)sei.m_projRegionHeight[i],            32,        "rwp_proj_region_height" );
      xWriteCode( (uint32_t)sei.m_rwpProjRegionTop[i],            32,        "rwp_proj_region_top" );
      xWriteCode( (uint32_t)sei.m_projRegionLeft[i],              32,        "rwp_proj_region_left" );
      xWriteCode( (uint32_t)sei.m_packedRegionWidth[i],           16,        "rwp_packed_region_width" );
      xWriteCode( (uint32_t)sei.m_packedRegionHeight[i],          16,        "rwp_packed_region_height" );
      xWriteCode( (uint32_t)sei.m_packedRegionTop[i],             16,        "rwp_packed_region_top" );
      xWriteCode( (uint32_t)sei.m_packedRegionLeft[i],            16,        "rwp_packed_region_left" );
      if( sei.m_rwpGuardBandFlag[i] )
      {
        xWriteCode( (uint32_t)sei.m_rwpLeftGuardBandWidth[i],     8,         "rwp_left_guard_band_width");
        xWriteCode( (uint32_t)sei.m_rwpRightGuardBandWidth[i],    8,         "rwp_right_guard_band_width");
        xWriteCode( (uint32_t)sei.m_rwpTopGuardBandHeight[i],     8,         "rwp_top_guard_band_height");
        xWriteCode( (uint32_t)sei. m_rwpBottomGuardBandHeight[i], 8,         "rwp_bottom_guard_band_height");
        xWriteFlag( sei.m_rwpGuardBandNotUsedForPredFlag[i],                 "rwp_guard_band_not_used_for_pred_flag" );
        for( int j=0; j < 4; j++ )
        {
          xWriteCode( (uint32_t)sei.m_rwpGuardBandType[i*4 + j],  3,         "rwp_guard_band_type");
        }
        xWriteCode( 0, 3,                                                    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

void SEIWriter::xWriteSEIGeneralizedCubemapProjection(const SEIGeneralizedCubemapProjection &sei,
                                                      int                                    SEIPrefixIndicationIdx)
{
  if (SEIPrefixIndicationIdx)
  {
    int numBits = 1;
    if (!sei.m_gcmpCancelFlag) 
    {
      numBits += 6;
    }
    xWriteCode(numBits - 1, 16, "num_bits_in_prefix_indication_minus1");
  }

  xWriteFlag( sei.m_gcmpCancelFlag,                           "gcmp_cancel_flag" );
  if (!sei.m_gcmpCancelFlag)
  {
    xWriteFlag( sei.m_gcmpPersistenceFlag,                    "gcmp_persistence_flag" );
    xWriteCode( sei.m_gcmpPackingType,                     3, "gcmp_packing_type" );
    xWriteCode( sei.m_gcmpMappingFunctionType,             2, "gcmp_mapping_function_type" );
    if (SEIPrefixIndicationIdx == 1)
    {
      return;
    }
    int numFace = sei.m_gcmpPackingType == 4 || sei.m_gcmpPackingType == 5 ? 5 : 6;
    for (int i = 0; i < numFace; i++)
    {
      xWriteCode( sei.m_gcmpFaceIndex[i],                  3, "gcmp_face_index" );
      xWriteCode( sei.m_gcmpFaceRotation[i],               2, "gcmp_face_rotation" );
      if (sei.m_gcmpMappingFunctionType == 2)
      {
        xWriteCode( sei.m_gcmpFunctionCoeffU[i],           7, "gcmp_function_coeff_u" );
        xWriteFlag( sei.m_gcmpFunctionUAffectedByVFlag[i],    "gcmp_function_u_affected_by_v_flag" );
        xWriteCode( sei.m_gcmpFunctionCoeffV[i],           7, "gcmp_function_coeff_v" );
        xWriteFlag( sei.m_gcmpFunctionVAffectedByUFlag[i],    "gcmp_function_v_affected_by_u_flag" );
      }
    }
    xWriteFlag( sei.m_gcmpGuardBandFlag,                      "gcmp_guard_band_flag" );
    if (sei.m_gcmpGuardBandFlag)
    {
      xWriteCode( sei.m_gcmpGuardBandType,                 3, "gcmp_guard_band_type" );
      xWriteFlag( sei.m_gcmpGuardBandBoundaryExteriorFlag,    "gcmp_guard_band_boundary_exterior_flag" );
      xWriteCode( sei.m_gcmpGuardBandSamplesMinus1,        4, "gcmp_guard_band_samples_minus1" );
    }
  }
}

void SEIWriter::xWriteSEIScalabilityDimensionInfo(const SEIScalabilityDimensionInfo &sei)
{
  xWriteCode(sei.m_sdiMaxLayersMinus1, 6,                           "sdi_max_layers_minus1");
  xWriteFlag(sei.m_sdiMultiviewInfoFlag,                            "sdi_multiview_info_flag");
  xWriteFlag(sei.m_sdiAuxiliaryInfoFlag,                            "sdi_auxiliary_info_flag");
  if (sei.m_sdiMultiviewInfoFlag || sei.m_sdiAuxiliaryInfoFlag)
  {
    if (sei.m_sdiMultiviewInfoFlag)
    {
      xWriteCode(sei.m_sdiViewIdLenMinus1, 4,                              "sdi_view_id_len_minus1");
    }
    for (int i = 0; i <= sei.m_sdiMaxLayersMinus1; i++)
    {
      xWriteCode(sei.m_sdiLayerId[i], 6,                                         "sdi_layer_id");
      if (sei.m_sdiMultiviewInfoFlag)
      {
        xWriteCode(sei.m_sdiViewIdVal[i], sei.m_sdiViewIdLenMinus1 + 1,       "sdi_view_id_val");
      }
      if (sei.m_sdiAuxiliaryInfoFlag)
      {
        xWriteCode(sei.m_sdiAuxId[i], 8,                           "sdi_aux_id");
        if (sei.m_sdiAuxId[i] > 0)
        {
          xWriteCode(sei.m_sdiNumAssociatedPrimaryLayersMinus1[i], 6,          "sdi_num_associated_primary_layers_minus1");
          for (int j = 0; j <= sei.m_sdiNumAssociatedPrimaryLayersMinus1[i]; j++)
          {
            xWriteCode(sei.m_sdiAssociatedPrimaryLayerIdx[i][j], 6,               "sdi_associated_primary_layer_idx");
          }
        }
      }
    }
  }
}

void SEIWriter::xWriteSEIMultiviewAcquisitionInfo(const SEIMultiviewAcquisitionInfo& sei)
{
  xWriteFlag( ( sei.m_maiIntrinsicParamFlag ? 1 : 0 ), "intrinsic_param_flag" );
  xWriteFlag( ( sei.m_maiExtrinsicParamFlag ? 1 : 0 ), "extrinsic_param_flag" );
  xWriteUvlc(   sei.m_maiNumViewsMinus1               , "num_views_minus1"           );
  if( sei.m_maiIntrinsicParamFlag )
  {
    xWriteFlag( ( sei.m_maiIntrinsicParamsEqualFlag ? 1 : 0 ), "intrinsic_params_equal_flag" );
    xWriteUvlc(   sei.m_maiPrecFocalLength                   , "prec_focal_length"           );
    xWriteUvlc(   sei.m_maiPrecPrincipalPoint                , "prec_principal_point"        );
    xWriteUvlc(   sei.m_maiPrecSkewFactor                    , "prec_skew_factor"            );

    for( int i = 0; i  <=  ( sei.m_maiIntrinsicParamsEqualFlag ? 0 : sei.m_maiNumViewsMinus1 ); i++ )
    {
      xWriteFlag( ( sei.m_maiSignFocalLengthX       [i] ? 1 : 0 ),                                         "sign_focal_length_x"        );
      xWriteCode(   sei.m_maiExponentFocalLengthX   [i]          , 6                                  ,    "exponent_focal_length_x"    );
      xWriteCode(   sei.m_maiMantissaFocalLengthX   [i]          , sei.getMantissaFocalLengthXLen( i ),    "mantissa_focal_length_x"    );
      xWriteFlag( ( sei.m_maiSignFocalLengthY       [i] ? 1 : 0 ),                                         "sign_focal_length_y"        );
      xWriteCode(   sei.m_maiExponentFocalLengthY   [i]          , 6                                  ,    "exponent_focal_length_y"    );
      xWriteCode(   sei.m_maiMantissaFocalLengthY   [i]          , sei.getMantissaFocalLengthYLen( i ),    "mantissa_focal_length_y"    );
      xWriteFlag( ( sei.m_maiSignPrincipalPointX    [i] ? 1 : 0 ),                                         "sign_principal_point_x"     );
      xWriteCode(   sei.m_maiExponentPrincipalPointX[i]          , 6,                                      "exponent_principal_point_x" );
      xWriteCode(   sei.m_maiMantissaPrincipalPointX[i]          , sei.getMantissaPrincipalPointXLen( i ), "mantissa_principal_point_x" );
      xWriteFlag( ( sei.m_maiSignPrincipalPointY    [i] ? 1 : 0 ),                                         "sign_principal_point_y"     );
      xWriteCode(   sei.m_maiExponentPrincipalPointY[i]          , 6,                                      "exponent_principal_point_y" );
      xWriteCode(   sei.m_maiMantissaPrincipalPointY[i]          , sei.getMantissaPrincipalPointYLen( i ), "mantissa_principal_point_y" );
      xWriteFlag( ( sei.m_maiSignSkewFactor         [i] ? 1 : 0 ),                                         "sign_skew_factor"           );
      xWriteCode(   sei.m_maiExponentSkewFactor     [i]          , 6,                                      "exponent_skew_factor"       );
      xWriteCode(   sei.m_maiMantissaSkewFactor     [i]          , sei.getMantissaSkewFactorLen( i )  ,    "mantissa_skew_factor"       );
    }
  }
  if( sei.m_maiExtrinsicParamFlag )
  {
    xWriteUvlc( sei.m_maiPrecRotationParam   , "prec_rotation_param"    );
    xWriteUvlc( sei.m_maiPrecTranslationParam, "prec_translation_param" );
    for( int i = 0; i  <=  sei.m_maiNumViewsMinus1; i++ )
    {
      for( int j = 0; j  <=  2; j++ )  /* row */
      {
        for( int k = 0; k  <=  2; k++ )  /* column */
        {
          xWriteFlag( ( sei.m_maiSignR    [i][j][k] ? 1 : 0 ),                                "sign_r"     );
          xWriteCode(   sei.m_maiExponentR[i][j][k]          , 6,                             "exponent_r" );
          xWriteCode(   sei.m_maiMantissaR[i][j][k]          , sei.getMantissaRLen( i,j,k ) , "mantissa_r" );
        }
        xWriteFlag( ( sei.m_maiSignT    [i][j] ? 1 : 0 ),                          "sign_t"     );
        xWriteCode(   sei.m_maiExponentT[i][j]          , 6,                       "exponent_t" );
        xWriteCode(   sei.m_maiMantissaT[i][j]          , sei.getMantissaTLen( i,j ),"mantissa_t" );
      }
    }
  }
};

void SEIWriter::xWriteSEIMultiviewViewPosition(const SEIMultiviewViewPosition& sei)
{
  xWriteUvlc(sei.m_mvpNumViewsMinus1, "num_views_minus1");
  for (int i = 0; i <= sei.m_mvpNumViewsMinus1; i++)
  {
    xWriteUvlc(sei.m_mvpViewPosition[i], "view_position");
  }
};

void SEIWriter::xWriteSEIAlphaChannelInfo( const SEIAlphaChannelInfo& sei)
{
  xWriteFlag( ( sei.m_aciCancelFlag ? 1 : 0 ), "alpha_channel_cancel_flag" );
  if( !sei.m_aciCancelFlag )
  {
    xWriteCode( sei.m_aciUseIdc, 3, "alpha_channel_use_idc" );
    xWriteCode( sei.m_aciBitDepthMinus8, 3, "alpha_channel_bit_depth_minus8" );
    xWriteCode( sei.m_aciTransparentValue, sei.m_aciBitDepthMinus8+9, "alpha_transparent_value" );
    xWriteCode( sei.m_aciOpaqueValue, sei.m_aciBitDepthMinus8+9, "alpha_opaque_value" );
    xWriteFlag( ( sei.m_aciIncrFlag ? 1 : 0 ), "alpha_channel_incr_flag" );
    xWriteFlag( ( sei.m_aciClipFlag ? 1 : 0 ), "alpha_channel_clip_flag" );
    if( sei.m_aciClipFlag )
    {
      xWriteFlag( ( sei.m_aciClipTypeFlag ? 1 : 0 ), "alpha_channel_clip_type_flag" );
    }
  }
};

void SEIWriter::xWriteSEIDepthRepresentationInfo( const SEIDepthRepresentationInfo& sei)
{
  xWriteFlag( ( sei.m_driZNearFlag ? 1 : 0 ), "z_near_flag" );
  xWriteFlag( ( sei.m_driZFarFlag ? 1 : 0 ), "z_far_flag" );
  xWriteFlag( ( sei.m_driDMinFlag ? 1 : 0 ), "d_min_flag" );
  xWriteFlag( ( sei.m_driDMaxFlag ? 1 : 0 ), "d_max_flag" );
  xWriteUvlc( sei.m_driDepthRepresentationType, "depth_representation_type" );
  if( sei.m_driDMinFlag || sei.m_driDMaxFlag )
  {
    xWriteUvlc( sei.m_driDisparityRefViewId, "disparity_ref_view_id" );
  }
  if( sei.m_driZNearFlag )
  {
    xWriteSEIDepthRepInfoElement(sei.m_driZNear);
  }
  if( sei.m_driZFarFlag )
  {
    xWriteSEIDepthRepInfoElement(sei.m_driZFar);
  }
  if( sei.m_driDMinFlag )
  {
    xWriteSEIDepthRepInfoElement(sei.m_driDMin);
  }
  if( sei.m_driDMaxFlag )
  {
    xWriteSEIDepthRepInfoElement(sei.m_driDMax);
  }

  if (sei.m_driDepthRepresentationType == 3)
  {
    xWriteUvlc( sei.m_driDepthNonlinearRepresentationNumMinus1, "depth_nonlinear_representation_num_minus1" );
    for( int i = 1; i  <=  sei.m_driDepthNonlinearRepresentationNumMinus1 + 1; i++ )
    {
      xWriteUvlc(sei.m_driDepthNonlinearRepresentationModel.at(i - 1),"depth_nonlinear_representation_model[ i ]");
    }
  }
}

void SEIWriter::xWriteSEIDepthRepInfoElement( double f )
{
  uint32_t x_sign, x_exp, x_mantissa,x_mantissa_len;
  if (f < 0)
  {
    f = f * (-1);
    x_sign = 1;
  }
  else
  {
    x_sign = 0;
  }
  int exponent=0;
  if(f >= 1)
  {
    while(f>=2)
    {
      exponent++;
      f = f/2;
    }
  }
  else
  {
    while (f<1)
    {
      exponent++;
      f = f*2;
    }
    exponent=-exponent;
  }

  int i;
  f = f -1;
  double s = 1;
  char s_mantissa[32];
  double thr=1.0/(4.0*(1<<30));

  if (f>=thr)
  {
    for(i=0;i<32;i++)
    {
      s /= 2;
      if(f>=s)
      {
        f = f-s;
        s_mantissa[i]=1;

        if (f<thr)
          break;
      }
      else
      {
        s_mantissa[i]=0;
      }
    }

    if (i<32)
      x_mantissa_len=i+1;
    else
      x_mantissa_len=32;

    x_mantissa=0;

    for(i=0;i<x_mantissa_len;i++)
    {
      if (s_mantissa[i]==1)
        x_mantissa += (1u)<<(x_mantissa_len-1-i) ;
    }

  }
  else
  {
    x_mantissa=0;
    x_mantissa_len=1;
  }

  assert(exponent>=-31 && exponent<= (1<<7)-32);
  x_exp=exponent+31;

  xWriteFlag( x_sign,                          "da_sign_flag" );
  xWriteCode( x_exp, 7 ,                       "da_exponent" );
  xWriteCode( x_mantissa_len-1, 5 ,            "da_mantissa_len_minus1" );
  xWriteCode( x_mantissa, x_mantissa_len ,     "da_mantissa" );
};

void SEIWriter::xWriteSEISubpictureLevelInfo(const SEISubpicureLevelInfo &sei)
{
  CHECK(sei.m_numRefLevels < 1, "SEISubpicureLevelInfo: numRefLevels must be greater than zero");
  CHECK(sei.m_numRefLevels != (int)sei.m_refLevelIdc.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of levels");
  if (sei.m_explicitFractionPresentFlag)
  {
    CHECK(sei.m_numRefLevels != (int)sei.m_refLevelFraction.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of fractions");
  }
  xWriteCode( (uint32_t)sei.m_numRefLevels - 1, 3,                            "sli_num_ref_levels_minus1");
  xWriteFlag(           sei.m_cbrConstraintFlag,                              "sli_cbr_constraint_flag");
  xWriteFlag(           sei.m_explicitFractionPresentFlag,                    "sli_explicit_fraction_present_flag");
  if (sei.m_explicitFractionPresentFlag)
  {
    xWriteUvlc(         sei.m_numSubpics -1 ,                                 "sli_num_subpics_minus1");
    xWriteCode( (uint32_t)sei.m_sliMaxSublayers - 1, 3,                       "sli_max_sublayers_minus1");
    xWriteFlag(           sei.m_sliSublayerInfoPresentFlag,                   "sli_sublayer_info_present_flag");
    while (!isByteAligned())
    {
      xWriteFlag(       0,                                                    "sli_alignment_zero_bit");
    }
  }

  for (int k = sei.m_sliSublayerInfoPresentFlag ? 0 : sei.m_sliMaxSublayers - 1; k < sei.m_sliMaxSublayers; k++)
  {
    for (int i = 0; i < sei.m_numRefLevels; i++)
    {
      xWriteCode((uint32_t)sei.m_nonSubpicLayersFraction[i][k], 8, "sli_non_subpic_layers_fraction[i][k]");
      xWriteCode((uint32_t)sei.m_refLevelIdc[i][k], 8, "sli_ref_level_idc[i][k]");
      if (sei.m_explicitFractionPresentFlag)
      {
        CHECK(sei.m_numSubpics != (int)sei.m_refLevelFraction[i].size(), "SEISubpicureLevelInfo: number of fractions differs from number of subpictures");
        for (int j = 0; j < sei.m_numSubpics; j++)
        {
          xWriteCode((uint32_t)sei.m_refLevelFraction[i][j][k], 8, "sli_ref_level_fraction_minus1[i][j][k]");
        }
      }
    }
  }
}

void SEIWriter::xWriteSEISampleAspectRatioInfo(const SEISampleAspectRatioInfo &sei)
{
  xWriteFlag( sei.m_sariCancelFlag,                                           "sari_cancel_flag" );
  if(!sei.m_sariCancelFlag)
  {
    xWriteFlag( sei.m_sariPersistenceFlag,                                    "sari_persistence_flag" );
    xWriteCode( (uint32_t)sei.m_sariAspectRatioIdc, 8,                        "sari_aspect_ratio_idc");
    if (sei.m_sariAspectRatioIdc == 255)
    {
      xWriteCode( (uint32_t)sei.m_sariSarWidth, 16,                           "sari_sar_width");
      xWriteCode( (uint32_t)sei.m_sariSarHeight, 16,                          "sari_sar_height");
    }
  }
}

void SEIWriter::xWriteSEIPhaseIndication(const SEIPhaseIndication& sei)
{
  xWriteCode((uint32_t)sei.m_horPhaseNum, 8, "hor_phase_num");
  xWriteCode((uint32_t)sei.m_horPhaseDenMinus1, 8, "hor_phase_den_minus1");
  xWriteCode((uint32_t)sei.m_verPhaseNum, 8, "ver_phase_num");
  xWriteCode((uint32_t)sei.m_verPhaseDenMinus1, 8, "ver_phase_den_minus1");
}

void SEIWriter::xWriteSEIUserDataRegistered(const SEIUserDataRegistered &sei)
{
  xWriteCode((sei.m_ituCountryCode>255) ? 0xff : sei.m_ituCountryCode, 8, "itu_t_t35_country_code");
  if (sei.m_ituCountryCode >= 255)
  {
    assert(sei.m_ituCountryCode < 255 + 256);
    xWriteCode(sei.m_ituCountryCode - 255, 8, "itu_t_t35_country_code_extension_byte");
  }
  for (uint32_t i = 0; i<sei.m_userData.size(); i++)
  {
    xWriteCode(sei.m_userData[i], 8, "itu_t_t35_payload_byte");
  }
}

void SEIWriter::xWriteSEIFilmGrainCharacteristics(const SEIFilmGrainCharacteristics &sei)
{
  xWriteFlag(sei.m_filmGrainCharacteristicsCancelFlag,        "fg_characteristics_cancel_flag");
  if (!sei.m_filmGrainCharacteristicsCancelFlag)
  {
    xWriteCode(sei.m_filmGrainModelId, 2,                     "fg_model_id");
    xWriteFlag(sei.m_separateColourDescriptionPresentFlag,    "fg_separate_colour_description_present_flag");
    if (sei.m_separateColourDescriptionPresentFlag)
    {
      xWriteCode(sei.m_filmGrainBitDepthLumaMinus8, 3,        "fg_bit_depth_luma_minus8");
      xWriteCode(sei.m_filmGrainBitDepthChromaMinus8, 3,      "fg_bit_depth_chroma_minus8");
      xWriteFlag(sei.m_filmGrainFullRangeFlag,                "fg_full_range_flag");
      xWriteCode(sei.m_filmGrainColourPrimaries, 8,           "fg_colour_primaries");
      xWriteCode(sei.m_filmGrainTransferCharacteristics, 8,   "fg_transfer_characteristics");
      xWriteCode(sei.m_filmGrainMatrixCoeffs, 8,              "fg_matrix_coeffs");
    }
    xWriteCode(sei.m_blendingModeId, 2,                       "fg_blending_mode_id");
    xWriteCode(sei.m_log2ScaleFactor, 4,                      "fg_log2_scale_factor");
    for (int c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t) cm.numIntensityIntervals;
      const uint32_t numModelValues = cm.numModelValues;
      xWriteFlag(sei.m_compModel[c].presentFlag && numIntensityIntervals>0 && numModelValues>0, "fg_comp_model_present_flag[c]");
    }
    for (uint32_t c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t) cm.numIntensityIntervals;
      const uint32_t numModelValues = cm.numModelValues;
      if (cm.presentFlag && numIntensityIntervals>0 && numModelValues>0)
      {
        assert(numIntensityIntervals <= 256);
        assert(numModelValues <= 256);
        xWriteCode(numIntensityIntervals - 1, 8,              "fg_num_intensity_intervals_minus1[c]");
        xWriteCode(numModelValues - 1, 3,                     "fg_num_model_values_minus1[c]");
        for (uint32_t interval = 0; interval<numIntensityIntervals; interval++)
        {
          const SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv = cm.intensityValues[interval];
          xWriteCode(cmiv.intensityIntervalLowerBound, 8,     "fg_intensity_interval_lower_bound[c][i]");
          xWriteCode(cmiv.intensityIntervalUpperBound, 8,     "fg_intensity_interval_upper_bound[c][i]");
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            xWriteSvlc(cmiv.compModelValue[j],                "fg_comp_model_value[c][i]");
          }
        }
      }
    } // for c
    xWriteFlag(sei.m_filmGrainCharacteristicsPersistenceFlag, "fg_characteristics_persistence_flag");
  } // cancel flag
}

void SEIWriter::xWriteSEIContentLightLevelInfo(const SEIContentLightLevelInfo& sei)
{
  xWriteCode( sei.m_maxContentLightLevel,    16, "clli_max_content_light_level"     );
  xWriteCode( sei.m_maxPicAverageLightLevel, 16, "clli_max_pic_average_light_level" );
}

void SEIWriter::xWriteSEIAmbientViewingEnvironment(const SEIAmbientViewingEnvironment& sei)
{
  xWriteCode(sei.m_ambientIlluminance, 32, "ambient_illuminance" );
  xWriteCode(sei.m_ambientLightX,      16, "ambient_light_x" );
  xWriteCode(sei.m_ambientLightY,      16, "ambient_light_y" );
}

void SEIWriter::xWriteSEIContentColourVolume(const SEIContentColourVolume &sei)
{
  xWriteFlag(sei.m_ccvCancelFlag, "ccv_cancel_flag");
  if (!sei.m_ccvCancelFlag)
  {
    xWriteFlag(sei.m_ccvPersistenceFlag, "ccv_persistence_flag");
    xWriteFlag(sei.m_ccvPrimariesPresentFlag, "ccv_primaries_present_flag");
    xWriteFlag(sei.m_ccvMinLuminanceValuePresentFlag, "ccv_min_luminance_value_present_flag");
    xWriteFlag(sei.m_ccvMaxLuminanceValuePresentFlag, "ccv_max_luminance_value_present_flag");
    xWriteFlag(sei.m_ccvAvgLuminanceValuePresentFlag, "ccv_avg_luminance_value_present_flag");

    if (sei.m_ccvPrimariesPresentFlag == true)
    {
      for (int i = 0; i < MAX_NUM_COMPONENT; i++)
      {
        xWriteSCode((int32_t)sei.m_ccvPrimariesX[i], 32, "ccv_primaries_x[i]");
        xWriteSCode((int32_t)sei.m_ccvPrimariesY[i], 32, "ccv_primaries_y[i]");
      }
    }

    if (sei.m_ccvMinLuminanceValuePresentFlag == true)
    {
      xWriteCode((uint32_t)sei.m_ccvMinLuminanceValue, 32, "ccv_min_luminance_value");
    }
    if (sei.m_ccvMaxLuminanceValuePresentFlag == true)
    {
      xWriteCode((uint32_t)sei.m_ccvMaxLuminanceValue, 32, "ccv_max_luminance_value");
    }
    if (sei.m_ccvAvgLuminanceValuePresentFlag == true)
    {
      xWriteCode((uint32_t)sei.m_ccvAvgLuminanceValue, 32, "ccv_avg_luminance_value");
    }
  }
}

void SEIWriter::xWriteSEIColourTransformInfo(const SEIColourTransformInfo& sei)
{
  bool colourTransformCancelFlag = 0;
  bool colourTransformPersistenceFlag = 0;

  xWriteUvlc(sei.m_id, "colour_transform_id");
  xWriteFlag(colourTransformCancelFlag, "colour_transform_cancel_flag");

  if (colourTransformCancelFlag == 0)
  {
    xWriteFlag(colourTransformPersistenceFlag, "colour_transform_persistence_flag");
    xWriteFlag(sei.m_signalInfoFlag, "colour_transform_video_signal_info_present_flag");

    if (sei.m_signalInfoFlag)
    {
      xWriteFlag(sei.m_fullRangeFlag, "colour_transform_full_range_flag");
      xWriteCode(sei.m_primaries, 8, "colour_transform_primaries");
      xWriteCode(sei.m_transferFunction, 8, "colour_transform_transfer_function");
      xWriteCode(sei.m_matrixCoefs, 8, "colour_transform_matrix_coefficients");
    }
    xWriteCode(sei.m_bitdepth - 8, 4, "colour_transform_bit_depth_minus8");
    xWriteCode(sei.m_log2NumberOfPointsPerLut - 1, 3, "colour_transform_log2_number_of_points_per_lut_minus1");
    xWriteFlag(sei.m_crossComponentFlag, "colour_transform_cross_comp_flag");
    if (sei.m_crossComponentFlag)
    {
      xWriteFlag(sei.m_crossComponentInferred, "colour_transform_cross_comp_inferred");
    }

    uint16_t lutCodingLength = 2 + sei.m_bitdepth - sei.m_log2NumberOfPointsPerLut;
    for (uint32_t j = 0; j < sei.m_lut[0].numLutValues; j++)
    {
      xWriteCode(sei.m_lut[0].lutValues[j], lutCodingLength, "colour_transform_lut[0][i]");
    }
    if (sei.m_crossComponentFlag == 0 || sei.m_crossComponentInferred == 0)
    {
      xWriteFlag(sei.m_numberChromaLutMinus1, "colour_transform_number_chroma_lut_minus1");
      for (uint32_t j = 0; j < sei.m_lut[1].numLutValues; j++)
      {
        xWriteCode(sei.m_lut[1].lutValues[j], lutCodingLength, "colour_transform_lut[1][i]");
      }
      if (sei.m_numberChromaLutMinus1 == 1)
      {
        for (uint32_t j = 0; j < sei.m_lut[2].numLutValues; j++)
        {
          xWriteCode(sei.m_lut[2].lutValues[j], lutCodingLength, "colour_transform_lut[2][i]");
        }
      }
    }
    else
    {
      xWriteCode(sei.m_chromaOffset, lutCodingLength, "colour_transform_chroma_offset");
    }
  }
}

void SEIWriter::xWriteSEIShutterInterval(const SEIShutterIntervalInfo &sei)
{
  xWriteCode(sei.m_siiTimeScale, 32, "sii_time_scale");
  xWriteFlag(sei.m_siiFixedSIwithinCLVS, "fixed_shutter_interval_within_clvs_flag");
  if (sei.m_siiFixedSIwithinCLVS)
  {
    xWriteCode(sei.m_siiNumUnitsInShutterInterval, 32, "sii_num_units_in_shutter_interval");
  }
  else
  {
    xWriteCode(sei.m_siiMaxSubLayersMinus1, 3, "sii_max_sub_layers_minus1");
    for (unsigned i = 0; i <= sei.m_siiMaxSubLayersMinus1; i++)
    {
      xWriteCode(sei.m_siiSubLayerNumUnitsInSI[i], 32, "sub_layer_num_units_in_shutter_interval[ i ]");
    }
  }
}

void SEIWriter::xWriteSEIProcessingOrder(OutputBitstream& bs, const SEIProcessingOrderInfo& sei)
{
  CHECK(sei.m_posPayloadType.size() < 2, "An SEI processing order SEI message shall contain at least two pairs sei_payloadType[i] and sei_processingOrder[i]");
  SEIMessages wrapSEI;
#if JVET_AF0061_ADDITION_PO_ID
  xWriteCode(sei.m_posId, 8, "po_sei_id");
#endif
  xWriteCode(sei.m_posNumMinus2, 8, "po_num_sei_message_minus2");
  for (uint32_t i = 0; i < ( sei.m_posNumMinus2 + 2 ); i++)
  {
    xWriteFlag(sei.m_posWrappingFlag[i], "po_sei_wrapping_flag[i]");
    xWriteFlag(sei.m_posImportanceFlag[i], "po_sei_importance_flag[i]");
#if !JVET_AF0310_PO_NESTING
    if (sei.m_posWrappingFlag[i])
    {
      xWriteCode(0, 6, "spo_sei_reserved_zero_6bits");
      wrapSEI = getSeisByType(sei.m_posWrapSeiMessages, SEI::PayloadType(sei.m_posPayloadType[i]));
      writeSEImessages(bs, wrapSEI, m_nestingHrd, true, 0);
    }
    else
    {
#endif
#if JVET_AF0062_MOVE_PO_SEI_PREFIX_FLAG
      xWriteCode(sei.m_posPayloadType[i], 13, "po_sei_payload_type[i]");
      xWriteFlag(sei.m_posPrefixFlag[i], "po_sei_prefix_flag[i]");
#else
      xWriteFlag(sei.m_posPrefixFlag[i], "po_sei_prefix_flag[i]");
      xWriteCode(sei.m_posPayloadType[i], 13, "po_sei_payload_type[i]");
#endif

#if !JVET_AF0310_PO_NESTING
      if (sei.m_posPrefixFlag[i])
    {
      xWriteCode((uint32_t)sei.m_posPrefixByte[i].size(), 8, "po_num_t35_byte[i]");
      for (uint32_t j = 0; j < sei.m_posPrefixByte[i].size(); j++)
      {
        xWriteCode(sei.m_posPrefixByte[i][j], 8, "po_t35_byte[i][j]");
      }
    }
    }
#endif
#if JVET_AF0310_PO_NESTING
    CHECK((i > 0) && (sei.m_posProcessingOrder[i] < sei.m_posProcessingOrder[i-1]) , "For i greater than 0, po_sei_processing_order[i] shall be greater than or equal to po_sei_processing_order[i-1]");
#endif
    xWriteCode(sei.m_posProcessingOrder[i], 8, "po_sei_processing_order[i]");
  }

#if JVET_AF0310_PO_NESTING
  for (uint32_t i = 0; i < ( sei.m_posNumMinus2 + 2 ); i++)
  {
    if (sei.m_posPrefixFlag[i])
    {
      xWriteCode(sei.m_posNumBitsInPrefix[i], 8, "po_num_bits_in_prefix_indication_minus1[i]");
      for (uint32_t j = 0; j < sei.m_posNumBitsInPrefix[i]; j += 8)
      {
        uint32_t numBits = (sei.m_posNumBitsInPrefix[i] - j) < 8 ? (sei.m_posNumBitsInPrefix[i] - j) : 8;
        for (int k = (int)numBits - 1; k >= 0; k--)
        {
          xWriteCode((sei.m_posPrefixByte[i][j>>3] >> k) & 1, 1, "po_sei_prefix_data_bit[i][j]");
        }
      }
      while (!isByteAligned())
      {
        xWriteCode(1, 1, "po_byte_alignment_bit_equal_to_one");
      }
    }
  }
#endif
}

#if JVET_AF0310_PO_NESTING
void SEIWriter::xWriteSEIProcessingOrderNesting(OutputBitstream& bs, const SEIProcessingOrderNesting& sei)
{
  SEIMessages wrapSEI;
  xWriteCode((uint32_t)(sei.m_ponTargetPoId.size() - 1), 8, "pon_num_po_ids_minus1");
  for (int i = 0; i < (int)sei.m_ponTargetPoId.size(); i++)
  {
    xWriteCode(sei.m_ponTargetPoId[i], 8, "pon_target_po_id[i]");
  }
  xWriteCode(sei.m_ponNumSeisMinus1, 8, "pon_num_seis_minus1");
  for (int i = 0; i <= sei.m_ponNumSeisMinus1; i++)
  {
    CHECK((i > 0) && (sei.m_ponProcessingOrder[i] < sei.m_ponProcessingOrder[i-1]) , "When i is greater than 0, pon_processing_order[i] shall be greater than or equal to pon_processing_order[i-1]");
    xWriteCode(sei.m_ponProcessingOrder[i], 8, "pon_processing_order[i]");
    wrapSEI = getSeisByType(sei.m_ponWrapSeiMessages, SEI::PayloadType(sei.m_ponPayloadType[i]));
    writeSEImessages(bs, wrapSEI, m_nestingHrd, true, 0);
  }
}
#endif

void SEIWriter::xWriteSEIConstrainedRaslIndication(const SEIConstrainedRaslIndication& /*sei*/)
{
  // intentionally empty
}

#if GREEN_METADATA_SEI_ENABLED
void SEIWriter::xWriteSEIGreenMetadataInfo(const SEIGreenMetadataInfo& sei)
{
  xWriteCode(sei.m_greenMetadataType, 8, "green_metadata_type");
  switch (sei.m_greenMetadataType)
  {
  case 0:
    xWriteCode(sei.m_periodType,4, "period_type");
    xWriteCode(sei.m_greenMetadataGranularityType,3, "granularity_type");
    xWriteCode(sei.m_greenMetadataExtendedRepresentation,1, "extended_representation_flag");
    
    if (sei.m_periodType == 2)
    {
      xWriteCode(sei.m_numSeconds, 16, "num_seconds");
    }
    else if (sei.m_periodType == 3)
    {
      xWriteCode(sei.m_numPictures, 16, "num_pictures");
    }
    
    if (sei.m_greenMetadataGranularityType == 0)
    {
      xWriteCode(sei.m_greenComplexityMetrics.portionNonZeroBlocksArea, 8, "portion_non_zero_blocks_area");
      xWriteCode(sei.m_greenComplexityMetrics.portionNonZeroTransformCoefficientsArea, 8, "portion_non_zero_transform_coefficients_area");
      xWriteCode(sei.m_greenComplexityMetrics.portionIntraPredictedBlocksArea, 8, "portion_intra_predicted_blocks_area");
      xWriteCode(sei.m_greenComplexityMetrics.portionDeblockingInstances, 8, "portion_deblocking_instances");
      xWriteCode(sei.m_greenComplexityMetrics.portionAlfInstances, 8, "portion_alf_instances");
      
      if(sei.m_greenMetadataExtendedRepresentation == 1)
      {
        if(sei.m_greenComplexityMetrics.portionNonZeroBlocksArea != 0)
        {
          xWriteCode(sei.m_greenComplexityMetrics.portionNonZero_4_8_16BlocksArea, 8, "portion_non_zero_4_8_16_blocks_area");
          xWriteCode(sei.m_greenComplexityMetrics.portionNonZero_32_64_128BlocksArea, 8, "portion_non_zero_32_64_128_blocks_area");
          xWriteCode(sei.m_greenComplexityMetrics.portionNonZero_256_512_1024BlocksArea, 8, "portion_non_zero_256_512_1024_blocks_area");
          xWriteCode(sei.m_greenComplexityMetrics.portionNonZero_2048_4096BlocksArea, 8, "portion_non_zero_2048_4096_blocks_area");
        }
        
        
        if(sei.m_greenComplexityMetrics.portionIntraPredictedBlocksArea < 255)
        {
          xWriteCode(sei.m_greenComplexityMetrics.portionBiAndGpmPredictedBlocksArea, 8,"portion_bi_and_gpm_predicted_blocks_area");
          xWriteCode(sei.m_greenComplexityMetrics.portionBdofBlocksArea, 8,"portion_bdof_blocks_area");
        }
        
        xWriteCode(sei.m_greenComplexityMetrics.portionSaoInstances, 8, "portion_sao_instances");
      }
    }
    
    break;
  case 1:
    int xsdSubpicNumberMinus1 = 0;
    xWriteCode(xsdSubpicNumberMinus1, 16, "xsd_subpic_number_minus1");
    for (int i = 0; i <= xsdSubpicNumberMinus1; i++)
    {
      int xsdMetricNumberMinus1 = -1;
      xWriteCode(sei.m_xsdSubPicIdc, 16, "xsd_subpic_idc[i]");
      std::vector <int> xsdMetricArray;
      if (sei.m_xsdMetricTypePSNR)
      {
        xsdMetricNumberMinus1++;
        xsdMetricArray.push_back(0);
      }
      if (sei.m_xsdMetricTypeSSIM)
      {
        xsdMetricNumberMinus1++;
        xsdMetricArray.push_back(1);
      }
  
      if (sei.m_xsdMetricTypeWPSNR)
      {
        xsdMetricNumberMinus1++;
        xsdMetricArray.push_back(2);
      }
  
      if (sei.m_xsdMetricTypeWSPSNR)
      {
        xsdMetricNumberMinus1++;
        xsdMetricArray.push_back(3);
      }
      
      xWriteCode(xsdMetricNumberMinus1, 8, "xsd_metric_number_minus1[i]");
      for (int j = 0; j <= xsdMetricNumberMinus1; j++)
      {
        if (xsdMetricArray[j] == 0)
        {
          xWriteCode(0, 8, "xsd_metric_type");
          xWriteCode(sei.m_xsdMetricValuePSNR, 16, "xsd_metric_type[i][j]");
        }
        else if (xsdMetricArray[j] == 1)
        {
          xWriteCode(1, 8, "xsd_metric_type");
          xWriteCode(sei.m_xsdMetricValueSSIM, 16, "xsd_metric_type[i][j]");
        }
        else if (xsdMetricArray[j] == 2)
        {
          xWriteCode(3, 8, "xsd_metric_type");
          xWriteCode(sei.m_xsdMetricValueWPSNR, 16, "xsd_metric_type[i][j]");
        }
        else if (xsdMetricArray[j] == 3)
        {
          xWriteCode(4, 8, "xsd_metric_type");
          xWriteCode(sei.m_xsdMetricValueWSPSNR, 16, "xsd_metric_type[i][j]");
        }
      }
    }
    break;
  }
}
#endif


void SEIWriter::xWriteSEINeuralNetworkPostFilterCharacteristics(const SEINeuralNetworkPostFilterCharacteristics &sei)
{
  xWriteCode(sei.m_purpose, 16, "nnpfc_purpose");
  xWriteUvlc(sei.m_id, "nnpfc_id");
  xWriteFlag(sei.m_baseFlag, "nnpfc_base_flag");
  xWriteUvlc(sei.m_modeIdc, "nnpfc_mode_idc");
  if (sei.m_modeIdc == POST_FILTER_MODE::URI)
  {
    while (!isByteAligned())
    {
      xWriteFlag(0, "nnpfc_alignment_zero_bit");
    }
    xWriteString(sei.m_uriTag, "nnpfc_uri_tag");
    xWriteString(sei.m_uri, "nnpfc_uri");
  }
  xWriteFlag(sei.m_propertyPresentFlag, "nnpfc_property_present_flag");
  if (sei.m_propertyPresentFlag)
  {
    xWriteUvlc(sei.m_numberInputDecodedPicturesMinus1, "nnpfc_number_of_input_pictures_minus1");

    if (sei.m_numberInputDecodedPicturesMinus1 > 0)
    {
      for (int i = 0; i <= sei.m_numberInputDecodedPicturesMinus1; ++i)
      {
        xWriteFlag(sei.m_inputPicOutputFlag[i], "nnpfc_input_pic_filtering_flag");
      }
      xWriteFlag(sei.m_absentInputPicZeroFlag, "nnpfc_absent_input_pic_zero_flag");
    }

    if((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      xWriteFlag(sei.m_outSubCFlag, "nnpfc_out_sub_c_flag");
    }
    if((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      xWriteCode(uint32_t(sei.m_outColourFormatIdc), 2, "nnpfc_out_colour_format_idc");
    }
    if((sei.m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      xWriteUvlc(sei.m_picWidthNumeratorMinus1, "nnpfc_pic_width_num_minus1");
      xWriteUvlc(sei.m_picWidthDenominatorMinus1, "nnpfc_pic_width_denom_minus1");
      xWriteUvlc(sei.m_picHeightNumeratorMinus1, "nnpfc_pic_height_num_minus1");
      xWriteUvlc(sei.m_picHeightDenominatorMinus1, "nnpfc_pic_height_denom_minus1");
    }

    if((sei.m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) != 0)
    {
      for (int i = 0; i < sei.m_numberInputDecodedPicturesMinus1; ++i)
      {
        xWriteUvlc(sei.m_numberInterpolatedPictures[i], "nnpfc_interpolated_pictures");
      }
    }

    xWriteFlag(sei.m_componentLastFlag, "nnpfc_component_last_flag");
    xWriteUvlc(sei.m_inpFormatIdc, "nnpfc_inp_format_idc");
    xWriteUvlc(sei.m_auxInpIdc, "nnpfc_auxiliary_inp_idc");
    xWriteUvlc(sei.m_inpOrderIdc, "nnpfc_inp_order_idc");
    if (sei.m_inpFormatIdc == 1)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        xWriteUvlc(sei.m_inpTensorBitDepthLumaMinus8, "nnpfc_inp_tensor_luma_bitdepth_minus8");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        xWriteUvlc(sei.m_inpTensorBitDepthChromaMinus8, "nnpfc_inp_tensor_chroma_bitdepth_minus8");
      }
    }
    xWriteUvlc(sei.m_outFormatIdc, "nnpfc_out_format_idc");
    xWriteUvlc(sei.m_outOrderIdc, "nnpfc_out_order_idc");
    if (sei.m_outFormatIdc == 1)
    {
      if (sei.m_outOrderIdc != 1)
      {
        xWriteUvlc(sei.m_outTensorBitDepthLumaMinus8, "nnpfc_out_tensor_luma_bitdepth_minus8");
      }
      if (sei.m_outOrderIdc != 0)
      {
        xWriteUvlc(sei.m_outTensorBitDepthChromaMinus8, "nnpfc_out_tensor_chroma_bitdepth_minus8");
      }
    }

    xWriteFlag(sei.m_sepColDescriptionFlag, "nnpfc_sep_col_desc_flag");

    if (sei.m_sepColDescriptionFlag)
    {
      xWriteCode(sei.m_colPrimaries, 8, "nnpfc_col_primaries");
      xWriteCode(sei.m_transCharacteristics, 8, "nnpfc_trans_characteristics");
      if (sei.m_outFormatIdc == 1)
      {
        xWriteCode(sei.m_matrixCoeffs, 8, "nnpfc_matrix_coeffs");
      }
    }
#if JVET_AD0067_INCLUDE_SYNTAX
    if (sei.m_sepColDescriptionFlag && (sei.m_outFormatIdc == 1))
    {
      xWriteFlag(sei.m_fullRangeFlag, "nnpfc_full_range_flag");
    }
#endif
    
    if (sei.m_outOrderIdc != 0)
    {   
      xWriteFlag(sei.m_chromaLocInfoPresentFlag, "nnpfc_chroma_loc_info_present_flag");
    }

    if(sei.m_chromaLocInfoPresentFlag)
    {
      xWriteUvlc(to_underlying(sei.m_chromaSampleLocTypeFrame), "nnpfc_chroma_sample_loc_type_frame");
    }
    
    xWriteUvlc(sei.m_overlap, "nnpfc_overlap");
    xWriteFlag(sei.m_constantPatchSizeFlag, "nnpfc_constant_patch_size_flag");
    if (sei.m_constantPatchSizeFlag)
    {
    xWriteUvlc(sei.m_patchWidthMinus1, "nnpfc_patch_width_minus1");
    xWriteUvlc(sei.m_patchHeightMinus1, "nnpfc_patch_height_minus1");
    }
    else
    {
      xWriteUvlc(sei.m_extendedPatchWidthCdDeltaMinus1, "extended_nnpfc_patch_width_cd_delta_minus1");
      xWriteUvlc(sei.m_extendedPatchHeightCdDeltaMinus1, "extended_nnpfc_patch_height_cd_delta_minus1");
    }
    xWriteUvlc(sei.m_paddingType, "nnpfc_padding_type");
    if (sei.m_paddingType == NNPC_PaddingType::FIXED_PADDING)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        xWriteUvlc(sei.m_lumaPadding, "nnpfc_luma_padding_val");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        xWriteUvlc(sei.m_cbPadding, "nnpfc_cb_padding_val");
        xWriteUvlc(sei.m_crPadding, "nnpfc_cr_padding_val");
      }
    }

    xWriteFlag(sei.m_complexityInfoPresentFlag, "nnpfc_complexity_info_present_flag");
    if (sei.m_complexityInfoPresentFlag)
    {
      xWriteCode(sei.m_parameterTypeIdc, 2, "nnpfc_parameter_type_idc");
      if (sei.m_parameterTypeIdc != 2)
      {
        xWriteCode(sei.m_log2ParameterBitLengthMinus3, 2, "nnpfc_log2_parameter_bit_length_minus3");
      }
      xWriteCode(sei.m_numParametersIdc, 6, "nnpfc_num_parameters_idc");
      xWriteUvlc(sei.m_numKmacOperationsIdc, "nnpfc_num_kmac_operations_idc");
      xWriteUvlc(sei.m_totalKilobyteSize, "nnpfc_total_kilobyte_size");
    }

    xWriteUvlc(0, "nnpfc_metadata_extension_num_bits");  // nnpfc_metadata_extension_num_bits shall be equal to 0 in the current edition 
  }
  if (sei.m_modeIdc == POST_FILTER_MODE::ISO_IEC_15938_17)
  {
    while (!isByteAligned())
    {
      xWriteFlag(0, "nnpfc_alignment_zero_bit");
    }
    for (long i = 0; i < sei.m_payloadLength; i++)
    {
      xWriteSCode(sei.m_payloadByte[i], 8, "nnpfc_payload_byte[i]");
    }
  }
}


void SEIWriter::xWriteSEINeuralNetworkPostFilterActivation(const SEINeuralNetworkPostFilterActivation &sei)
{
  xWriteUvlc(sei.m_targetId, "nnpfa_target_id");
  xWriteFlag(sei.m_cancelFlag, "nnpfa_cancel_flag");
  if(!sei.m_cancelFlag)
  {
    xWriteFlag(sei.m_persistenceFlag, "nnpfa_persistence_flag");
    xWriteFlag(sei.m_targetBaseFlag, "nnpfa_target_base_flag");
    xWriteFlag(sei.m_noPrevCLVSFlag, "nnpfa_no_prev_clvs_flag");
    if (sei.m_persistenceFlag)
    {
      xWriteFlag(sei.m_noFollCLVSFlag, "nnpfa_no_foll_clvs_flag");
    }
    xWriteUvlc((uint32_t)sei.m_outputFlag.size(), "nnpfa_num_output_entries");
    for (uint32_t i = 0; i < (uint32_t)sei.m_outputFlag.size(); i++)
    {
      xWriteFlag(sei.m_outputFlag[i], "nnpfa_output_flag");
    }
  }
}

void SEIWriter::xWriteSEIPostFilterHint(const SEIPostFilterHint &sei)
{
  xWriteFlag(sei.m_filterHintCancelFlag, "filter_hint_cancel_flag");
  if (sei.m_filterHintCancelFlag == false)
  {
    xWriteFlag(sei.m_filterHintPersistenceFlag, "filter_hint_persistence_flag");
    xWriteUvlc(sei.m_filterHintSizeY, "filter_hint_size_y");
    xWriteUvlc(sei.m_filterHintSizeX, "filter_hint_size_x");
    xWriteCode(sei.m_filterHintType, 2, "filter_hint_type");
    xWriteFlag(sei.m_filterHintChromaCoeffPresentFlag, "filter_hint_chroma_coeff_present_flag");

    CHECK(!(sei.m_filterHintValues.size() == ((sei.m_filterHintChromaCoeffPresentFlag ? 3 : 1) * sei.m_filterHintSizeX * sei.m_filterHintSizeY)), "The number of filter coefficient shall match the matrix size and considering whether filters for chroma is present of not");
    for (uint32_t i = 0; i < sei.m_filterHintValues.size(); i++)
    {
      xWriteSvlc(sei.m_filterHintValues[i], "filter_hint_value[][][]");
    }
  }
}
//! \}
