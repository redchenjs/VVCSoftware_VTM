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

/** \file     SEI.cpp
    \brief    helper functions for SEI handling
*/

#include "CommonDef.h"
#include "SEI.h"

#include "dtrace_next.h"

#if ENABLE_TRACING
void xTraceSEIHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== SEI message ===========\n" );
}

void xTraceSEIMessageType( SEI::PayloadType payloadType )
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== %s SEI message ===========\n", SEI::getSEIMessageString( payloadType ) );
}
#endif

SEIMessages getSeisByType(const SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  for (SEIMessages::const_iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
    }
  }
  return result;
}

SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  SEIMessages::iterator it=seiList.begin();
  while ( it!=seiList.end() )
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
      it = seiList.erase(it);
    }
    else
    {
      it++;
    }
  }
  return result;
}


void deleteSEIs (SEIMessages &seiList)
{
  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    delete (*it);
  }
  seiList.clear();
}

void SEIBufferingPeriod::copyTo (SEIBufferingPeriod& target) const
{
  target.m_bpNalCpbParamsPresentFlag = m_bpNalCpbParamsPresentFlag;
  target.m_bpVclCpbParamsPresentFlag = m_bpVclCpbParamsPresentFlag;
  target.m_initialCpbRemovalDelayLength = m_initialCpbRemovalDelayLength;
  target.m_cpbRemovalDelayLength = m_cpbRemovalDelayLength;
  target.m_dpbOutputDelayLength = m_dpbOutputDelayLength;
  target.m_duCpbRemovalDelayIncrementLength = m_duCpbRemovalDelayIncrementLength;
  target.m_dpbOutputDelayDuLength = m_dpbOutputDelayDuLength;
  target.m_concatenationFlag = m_concatenationFlag;
  target.m_auCpbRemovalDelayDelta = m_auCpbRemovalDelayDelta;
  target.m_cpbRemovalDelayDeltasPresentFlag =  m_cpbRemovalDelayDeltasPresentFlag;
  target.m_numCpbRemovalDelayDeltas = m_numCpbRemovalDelayDeltas;
  target.m_bpMaxSubLayers = m_bpMaxSubLayers;
  ::memcpy(target.m_initialCpbRemovalDelay, m_initialCpbRemovalDelay, sizeof(m_initialCpbRemovalDelay));
  ::memcpy(target.m_initialCpbRemovalOffset, m_initialCpbRemovalOffset, sizeof(m_initialCpbRemovalOffset));
  ::memcpy(target.m_cpbRemovalDelayDelta, m_cpbRemovalDelayDelta, sizeof(m_cpbRemovalDelayDelta));
  target.m_bpCpbCnt = m_bpCpbCnt;
  target.m_bpDecodingUnitHrdParamsPresentFlag = m_bpDecodingUnitHrdParamsPresentFlag;
  target.m_decodingUnitCpbParamsInPicTimingSeiFlag = m_decodingUnitCpbParamsInPicTimingSeiFlag;
  target.m_decodingUnitDpbDuParamsInPicTimingSeiFlag = m_decodingUnitDpbDuParamsInPicTimingSeiFlag;
  target.m_sublayerInitialCpbRemovalDelayPresentFlag = m_sublayerInitialCpbRemovalDelayPresentFlag;
  target.m_concatenationFlag = m_concatenationFlag;
  target.m_maxInitialRemovalDelayForConcatenation = m_maxInitialRemovalDelayForConcatenation;
  target.m_sublayerDpbOutputOffsetsPresentFlag = m_sublayerDpbOutputOffsetsPresentFlag;
  ::memcpy(target.m_dpbOutputTidOffset, m_dpbOutputTidOffset, sizeof(m_dpbOutputTidOffset));
  target.m_altCpbParamsPresentFlag = m_altCpbParamsPresentFlag;
}

void SEIPictureTiming::copyTo (SEIPictureTiming& target) const
{
  ::memcpy(target.m_auCpbRemovalDelay, m_auCpbRemovalDelay, sizeof(m_auCpbRemovalDelay));
  ::memcpy(target.m_ptSubLayerDelaysPresentFlag, m_ptSubLayerDelaysPresentFlag, sizeof(m_ptSubLayerDelaysPresentFlag));
  ::memcpy(target.m_duCommonCpbRemovalDelayMinus1, m_duCommonCpbRemovalDelayMinus1, sizeof(m_duCommonCpbRemovalDelayMinus1));
  ::memcpy(target.m_cpbRemovalDelayDeltaEnabledFlag, m_cpbRemovalDelayDeltaEnabledFlag, sizeof(m_cpbRemovalDelayDeltaEnabledFlag));
  ::memcpy(target.m_cpbRemovalDelayDeltaIdx, m_cpbRemovalDelayDeltaIdx, sizeof(m_cpbRemovalDelayDeltaIdx));
  target.m_picDpbOutputDelay = m_picDpbOutputDelay;
  target.m_picDpbOutputDuDelay = m_picDpbOutputDuDelay;
  target.m_numDecodingUnitsMinus1 = m_numDecodingUnitsMinus1;
  target.m_duCommonCpbRemovalDelayFlag = m_duCommonCpbRemovalDelayFlag;

  target.m_numNalusInDuMinus1 = m_numNalusInDuMinus1;
  target.m_duCpbRemovalDelayMinus1 = m_duCpbRemovalDelayMinus1;
  target.m_cpbAltTimingInfoPresentFlag = m_cpbAltTimingInfoPresentFlag;
  target.m_nalCpbAltInitialRemovalDelayDelta  = m_nalCpbAltInitialRemovalDelayDelta;
  target.m_nalCpbAltInitialRemovalOffsetDelta = m_nalCpbAltInitialRemovalOffsetDelta;
  target.m_nalCpbDelayOffset = m_nalCpbDelayOffset;
  target.m_nalCpbDelayOffset = m_nalCpbDelayOffset;
  target.m_vclCpbAltInitialRemovalDelayDelta  = m_vclCpbAltInitialRemovalDelayDelta;
  target.m_vclCpbAltInitialRemovalOffsetDelta = m_vclCpbAltInitialRemovalOffsetDelta;
  target.m_vclCpbDelayOffset = m_vclCpbDelayOffset;
  target.m_vclCpbDelayOffset = m_vclCpbDelayOffset;
}

bool SEIScalabilityDimensionInfo::isSDISameContent(SEIScalabilityDimensionInfo* sdiB)
{
  if (!sdiB)
  {
    return false;
  }
  if (m_sdiNumViews != sdiB->m_sdiNumViews)
  {
    return false;
  }
  if (m_sdiMaxLayersMinus1 != sdiB->m_sdiMaxLayersMinus1)
  {
    return false;
  }
  if (m_sdiMultiviewInfoFlag != sdiB->m_sdiMultiviewInfoFlag)
  {
    return false;
  }
  if (m_sdiAuxiliaryInfoFlag != sdiB->m_sdiAuxiliaryInfoFlag)
  {
    return false;
  }
  if (m_sdiMultiviewInfoFlag || m_sdiAuxiliaryInfoFlag)
  {
    if (m_sdiMultiviewInfoFlag)
    {
      if (m_sdiViewIdLenMinus1 != sdiB->m_sdiViewIdLenMinus1)
      {
        return false;
      }
    }
    for (int i = 0; i <= m_sdiMaxLayersMinus1; i++)
    {
      if (m_sdiMultiviewInfoFlag)
      {
        if (m_sdiViewIdVal[i] != sdiB->m_sdiViewIdVal[i])
        {
          return false;
        }
      }
      if (m_sdiAuxiliaryInfoFlag)
      {
        if (m_sdiAuxId[i] != sdiB->m_sdiAuxId[i])
        {
          return false;
        }
        if (m_sdiAuxId[i] > 0)
        {
          if (m_sdiNumAssociatedPrimaryLayersMinus1[i] != sdiB->m_sdiNumAssociatedPrimaryLayersMinus1[i])
          {
            return false;
          }
          for (int j = 0; j <= m_sdiNumAssociatedPrimaryLayersMinus1[i]; j++)
          {
            if (m_sdiAssociatedPrimaryLayerIdx[i][j] != sdiB->m_sdiAssociatedPrimaryLayerIdx[i][j])
            {
              return false;
            }
          }
        }
      }
    }
  }
  return true;
}

uint32_t SEIMultiviewAcquisitionInfo::getMantissaFocalLengthXLen( int i ) const
{
  return xGetSyntaxElementLen( m_maiExponentFocalLengthX[i], m_maiPrecFocalLength, m_maiMantissaFocalLengthX[ i ] );
}

uint32_t SEIMultiviewAcquisitionInfo::getMantissaFocalLengthYLen( int i ) const
{
  return xGetSyntaxElementLen( m_maiExponentFocalLengthY[i], m_maiPrecFocalLength, m_maiMantissaFocalLengthY[ i ]  );
}


uint32_t SEIMultiviewAcquisitionInfo::getMantissaPrincipalPointXLen( int i ) const
{
  return xGetSyntaxElementLen( m_maiExponentPrincipalPointX[i], m_maiPrecPrincipalPoint, m_maiMantissaPrincipalPointX[ i ]  );
}

uint32_t SEIMultiviewAcquisitionInfo::getMantissaPrincipalPointYLen( int i ) const
{
  return xGetSyntaxElementLen( m_maiExponentPrincipalPointY[i], m_maiPrecPrincipalPoint, m_maiMantissaPrincipalPointY[ i ] );
}

uint32_t SEIMultiviewAcquisitionInfo::getMantissaSkewFactorLen( int i ) const
{
  return xGetSyntaxElementLen( m_maiExponentSkewFactor[ i ], m_maiPrecSkewFactor, m_maiMantissaSkewFactor[ i ] );
}

uint32_t SEIMultiviewAcquisitionInfo::getMantissaRLen( int i, int j, int k ) const
{
  return xGetSyntaxElementLen( m_maiExponentR[ i ][ j ][ k ], m_maiPrecRotationParam, m_maiMantissaR[ i ][ j] [ k ] );
}

uint32_t SEIMultiviewAcquisitionInfo::getMantissaTLen( int i, int j ) const
{
  return xGetSyntaxElementLen( m_maiExponentT[ i ][ j ], m_maiPrecTranslationParam, m_maiMantissaT[ i ][ j ] );
}
uint32_t SEIMultiviewAcquisitionInfo::xGetSyntaxElementLen( int expo, int prec, int val ) const
{
  uint32_t len;
  if( expo == 0 )
  {
    len = std::max(0, prec - 30 );
  }
  else
  {
    len = std::max( 0, expo + prec - 31 );
  }

  assert( val >= 0 );
  assert( val <= ( ( 1 << len )- 1) );
  return len;
}

bool SEIMultiviewAcquisitionInfo::isMAISameContent(SEIMultiviewAcquisitionInfo *maiB)
{
  if (!maiB)
  {
    return false;
  }
  if (m_maiIntrinsicParamFlag != maiB->m_maiIntrinsicParamFlag)
  {
    return false;
  }
  if (m_maiExtrinsicParamFlag != maiB->m_maiExtrinsicParamFlag)
  {
    return false;
  }
  if (m_maiNumViewsMinus1 != maiB->m_maiNumViewsMinus1)
  {
    return false;
  }
  if (m_maiIntrinsicParamFlag)
  {
    if (m_maiIntrinsicParamsEqualFlag != maiB->m_maiIntrinsicParamsEqualFlag)
    {
      return false;
    }
    if (m_maiPrecFocalLength != maiB->m_maiPrecFocalLength)
    {
      return false;
    }
    if (m_maiPrecPrincipalPoint != maiB->m_maiPrecPrincipalPoint)
    {
      return false;
    }
    if (m_maiPrecSkewFactor != maiB->m_maiPrecSkewFactor)
    {
      return false;
    }
    for (int i = 0; i <= (m_maiIntrinsicParamsEqualFlag ? 0 : m_maiNumViewsMinus1); i++)
    {
      if (m_maiSignFocalLengthX[i] != maiB->m_maiSignFocalLengthX[i])
      {
        return false;
      }
      if (m_maiExponentFocalLengthX[i] != maiB->m_maiExponentFocalLengthX[i])
      {
        return false;
      }
      if (m_maiMantissaFocalLengthX[i] != maiB->m_maiMantissaFocalLengthX[i])
      {
        return false;
      }
      if (m_maiSignFocalLengthY[i] != maiB->m_maiSignFocalLengthY[i])
      {
        return false;
      }
      if (m_maiExponentFocalLengthY[i] != maiB->m_maiExponentFocalLengthY[i])
      {
        return false;
      }
      if (m_maiMantissaFocalLengthY[i] != maiB->m_maiMantissaFocalLengthY[i])
      {
        return false;
      }
      if (m_maiSignPrincipalPointX[i] != maiB->m_maiSignPrincipalPointX[i])
      {
        return false;
      }
      if (m_maiExponentPrincipalPointX[i] != maiB->m_maiExponentPrincipalPointX[i])
      {
        return false;
      }
      if (m_maiMantissaPrincipalPointX[i] != maiB->m_maiMantissaPrincipalPointX[i])
      {
        return false;
      }
      if (m_maiSignPrincipalPointY[i] != maiB->m_maiSignPrincipalPointY[i])
      {
        return false;
      }
      if (m_maiExponentPrincipalPointY[i] != maiB->m_maiExponentPrincipalPointY[i])
      {
        return false;
      }
      if (m_maiMantissaPrincipalPointY[i] != maiB->m_maiMantissaPrincipalPointY[i])
      {
        return false;
      }
      if (m_maiSignSkewFactor[i] != maiB->m_maiSignSkewFactor[i])
      {
        return false;
      }
      if (m_maiExponentSkewFactor[i] != maiB->m_maiExponentSkewFactor[i])
      {
        return false;
      }
      if (m_maiMantissaSkewFactor[i] != maiB->m_maiMantissaSkewFactor[i])
      {
        return false;
      }
    }
  }
  if (m_maiExtrinsicParamFlag)
  {
    if (m_maiPrecRotationParam != maiB->m_maiPrecRotationParam)
    {
      return false;
    }
    if (m_maiPrecTranslationParam != maiB->m_maiPrecTranslationParam)
    {
      return false;
    }
    for (int i = 0; i <= m_maiNumViewsMinus1; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        for (int k = 0; k < 3; k++)
        {
          if (m_maiSignR[i][j][k] != maiB->m_maiSignR[i][j][k])
          {
            return false;
          }
          if (m_maiExponentR[i][j][k] != maiB->m_maiExponentR[i][j][k])
          {
            return false;
          }
          if (m_maiMantissaR[i][j][k] != maiB->m_maiMantissaR[i][j][k])
          {
            return false;
          }
        }
        if (m_maiSignT[i][j] != maiB->m_maiSignT[i][j])
        {
          return false;
        }
        if (m_maiExponentT[i][j] != maiB->m_maiExponentT[i][j])
        {
          return false;
        }
        if (m_maiMantissaT[i][j] != maiB->m_maiMantissaT[i][j])
        {
          return false;
        }
      }
    }
  }
  return true;
}

SEIManifest::SEIManifestDescription SEIManifest::getSEIMessageDescription(const PayloadType payloadType)
{
  std::vector<PayloadType> necessary = { PayloadType::FRAME_PACKING, PayloadType::EQUIRECTANGULAR_PROJECTION,
                                         PayloadType::GENERALIZED_CUBEMAP_PROJECTION, PayloadType::SPHERE_ROTATION,
                                         PayloadType::REGION_WISE_PACKING };

  std::vector<PayloadType> undetermined = { PayloadType::USER_DATA_REGISTERED_ITU_T_T35,
                                            PayloadType::USER_DATA_UNREGISTERED };

  for (auto pt : necessary) 
  {
    if (payloadType == pt) 
    {
      return NECESSARY_SEI_MESSAGE;
    }
  }
  for (auto pt: undetermined)
  {
    if (payloadType == pt) 
    {
      return UNDETERMINED_SEI_MESSAGE;
    } 
  }
  return UNNECESSARY_SEI_MESSAGE;
}

uint8_t SEIPrefixIndication::getNumsOfSeiPrefixIndications(const SEI *sei)
{
  PayloadType payloadType = sei->payloadType();
  CHECK((payloadType == PayloadType::SEI_MANIFEST), "SEI_SPI should not include SEI_manfest");
  CHECK((payloadType == PayloadType::SEI_PREFIX_INDICATION), "SEI_SPI should not include itself");

  //Unable to determine how many indicators are needed, it will be determined in xWriteSEIPrefixIndication() return 3
  std::vector<PayloadType> indicationN = { PayloadType::REGION_WISE_PACKING };
  // Need two indicators to finish writing the SEI prefix indication message(return 2)
  std::vector<PayloadType> indication2 = { PayloadType::SPHERE_ROTATION };

  for (auto pt: indicationN)
  {
    if (payloadType == pt) 
    {
      return 3;
    }
  }
  for (auto pt: indication2)
  {
    if (payloadType == pt)
    {
      return 2;
    }
  }
  return 1;
}

bool SEIMultiviewViewPosition::isMVPSameContent(SEIMultiviewViewPosition *mvpB)
{
  if (!mvpB)
  {
    return false;
  }
  if (m_mvpNumViewsMinus1 != mvpB->m_mvpNumViewsMinus1)
  {
    return false;
  }
  for (int i = 0; i <= m_mvpNumViewsMinus1; i++)
  {
    if (m_mvpViewPosition[i] != mvpB->m_mvpViewPosition[i])
    {
      return false;
    }
  }
  return true;
}

// Static member
static const std::map<SEI::PayloadType, const char *> payloadTypeStrings = {
  { SEI::PayloadType::BUFFERING_PERIOD, "Buffering period" },
  { SEI::PayloadType::PICTURE_TIMING, "Picture timing" },
  { SEI::PayloadType::FILLER_PAYLOAD, "Filler payload" },
  { SEI::PayloadType::USER_DATA_REGISTERED_ITU_T_T35, "User data registered" },
  { SEI::PayloadType::USER_DATA_UNREGISTERED, "User data unregistered" },
  { SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS, "Film grain characteristics" },
  { SEI::PayloadType::FRAME_PACKING, "Frame packing arrangement" },
  { SEI::PayloadType::DISPLAY_ORIENTATION, "Display orientation" },
  { SEI::PayloadType::GREEN_METADATA, "Green metadata" },
  { SEI::PayloadType::PARAMETER_SETS_INCLUSION_INDICATION, "Parameter sets inclusion indication" },
  { SEI::PayloadType::DECODING_UNIT_INFO, "Decoding unit information" },
  { SEI::PayloadType::SCALABLE_NESTING, "Scalable nesting" },
  { SEI::PayloadType::DECODED_PICTURE_HASH, "Decoded picture hash" },
  { SEI::PayloadType::DEPENDENT_RAP_INDICATION, "Dependent RAP indication" },
  { SEI::PayloadType::MASTERING_DISPLAY_COLOUR_VOLUME, "Mastering display colour volume" },
  { SEI::PayloadType::ALTERNATIVE_TRANSFER_CHARACTERISTICS, "Alternative transfer characteristics" },
  { SEI::PayloadType::CONTENT_LIGHT_LEVEL_INFO, "Content light level information" },
  { SEI::PayloadType::AMBIENT_VIEWING_ENVIRONMENT, "Ambient viewing environment" },
  { SEI::PayloadType::CONTENT_COLOUR_VOLUME, "Content colour volume" },
  { SEI::PayloadType::COLOUR_TRANSFORM_INFO, "Colour transform information" },
  { SEI::PayloadType::EQUIRECTANGULAR_PROJECTION, "Equirectangular projection" },
  { SEI::PayloadType::SPHERE_ROTATION, "Sphere rotation" },
  { SEI::PayloadType::REGION_WISE_PACKING, "Region wise packing information" },
  { SEI::PayloadType::OMNI_VIEWPORT, "Omni viewport" },
  { SEI::PayloadType::GENERALIZED_CUBEMAP_PROJECTION, "Generalized cubemap projection" },
  { SEI::PayloadType::ALPHA_CHANNEL_INFO, "Alpha channel information" },
  { SEI::PayloadType::DEPTH_REPRESENTATION_INFO, "Depth representation information" },
  { SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO, "Multiview acquisition information" },
  { SEI::PayloadType::MULTIVIEW_VIEW_POSITION, "Multiview view position" },
  { SEI::PayloadType::SAMPLE_ASPECT_RATIO_INFO, "Sample aspect ratio information" },
  { SEI::PayloadType::SUBPICTURE_LEVEL_INFO, "Subpicture level information" },
  { SEI::PayloadType::ANNOTATED_REGIONS, "Annotated Region" },
  { SEI::PayloadType::SCALABILITY_DIMENSION_INFO, "Scalability dimension information" },
  { SEI::PayloadType::EXTENDED_DRAP_INDICATION, "Extended DRAP indication" },
  { SEI::PayloadType::SEI_MANIFEST, "SEI manifest" },
  { SEI::PayloadType::SEI_PREFIX_INDICATION, "SEI prefix indication" },
  { SEI::PayloadType::CONSTRAINED_RASL_ENCODING, "Constrained RASL encoding" },
  { SEI::PayloadType::VDI_SEI_ENVELOPE, "Video decoding interface SEI envelope" },
  { SEI::PayloadType::SHUTTER_INTERVAL_INFO, "Shutter interval information" },
  { SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS, "Neural network post-filter characteristics" },
  { SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_ACTIVATION, "Neural network post-filter activation" },
  { SEI::PayloadType::PHASE_INDICATION, "Phase Indication" },
  { SEI::PayloadType::SEI_PROCESSING_ORDER, "SEI messages Processing order" },
};

const char *SEI::getSEIMessageString(SEI::PayloadType payloadType)
{
  auto p = payloadTypeStrings.find(payloadType);
  if (p != payloadTypeStrings.end())
  {
    return p->second;
  }
  else
  {
    return "Unknown";
  }
}
