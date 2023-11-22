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

#include "CommonLib/CommonDef.h"
#include "CommonLib/SEI.h"
#include "EncGOP.h"
#include "EncLib.h"
#include <fstream>

uint32_t calcMD5(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcCRC(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcChecksum(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
std::string hashToString(const PictureHash &digest, int numChar);

//! \ingroup EncoderLib
//! \{

void SEIEncoder::initSEIFramePacking(SEIFramePacking *seiFramePacking, int currPicNum)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiFramePacking != nullptr), "Unspecified error");

  seiFramePacking->m_arrangementId = m_pcCfg->getFramePackingArrangementSEIId();
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType = m_pcCfg->getFramePackingArrangementSEIType();
  CHECK(!((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) ), "Unspecified error");
  seiFramePacking->m_quincunxSamplingFlag = m_pcCfg->getFramePackingArrangementSEIQuincunx();
  seiFramePacking->m_contentInterpretationType = m_pcCfg->getFramePackingArrangementSEIInterpretation();
  seiFramePacking->m_spatialFlippingFlag = 0;
  seiFramePacking->m_frame0FlippedFlag = 0;
  seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && (currPicNum&1) );
  seiFramePacking->m_frame0SelfContainedFlag = 0;
  seiFramePacking->m_frame1SelfContainedFlag = 0;
  seiFramePacking->m_frame0GridPositionX = 0;
  seiFramePacking->m_frame0GridPositionY = 0;
  seiFramePacking->m_frame1GridPositionX = 0;
  seiFramePacking->m_frame1GridPositionY = 0;
  seiFramePacking->m_arrangementReservedByte = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio = 0;
}


void SEIEncoder::initSEIParameterSetsInclusionIndication(SEIParameterSetsInclusionIndication* seiParameterSetsInclusionIndication)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiParameterSetsInclusionIndication != nullptr), "Unspecified error");

  seiParameterSetsInclusionIndication->m_selfContainedClvsFlag = m_pcCfg->getSelfContainedClvsFlag();
}

void SEIEncoder::initSEIBufferingPeriod(SEIBufferingPeriod *bufferingPeriodSEI, bool noLeadingPictures)
{
  CHECK(!(m_isInitialized), "bufferingPeriodSEI already initialized");
  CHECK(!(bufferingPeriodSEI != nullptr), "Need a bufferingPeriodSEI for initialization (got nullptr)");

  uint32_t uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
  bufferingPeriodSEI->m_bpNalCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpVclCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpMaxSubLayers = m_pcCfg->getMaxTempLayer() ;
  bufferingPeriodSEI->m_bpCpbCnt = 1;
  for(int i=0; i < bufferingPeriodSEI->m_bpMaxSubLayers; i++)
  {
    for(int j=0; j < bufferingPeriodSEI->m_bpCpbCnt; j++)
    {
      bufferingPeriodSEI->m_initialCpbRemovalDelay[i][j][0] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalDelay[i][j][1] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[i][j][0] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[i][j][1] = uiInitialCpbRemovalDelay;
    }
  }
  // We don't set concatenation_flag here. max_initial_removal_delay_for_concatenation depends on the usage scenario.
  // The parameters could be added to config file, but as long as the initialisation of generic buffering parameters is
  // not controllable, it does not seem to make sense to provide settings for these.
  bufferingPeriodSEI->m_concatenationFlag = false;
  bufferingPeriodSEI->m_maxInitialRemovalDelayForConcatenation = uiInitialCpbRemovalDelay;

  bufferingPeriodSEI->m_bpDecodingUnitHrdParamsPresentFlag = m_pcCfg->getNoPicPartitionFlag() == false;
  bufferingPeriodSEI->m_decodingUnitCpbParamsInPicTimingSeiFlag = !m_pcCfg->getDecodingUnitInfoSEIEnabled();

  bufferingPeriodSEI->m_initialCpbRemovalDelayLength = 16;                  // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  // Note: The following parameters require some knowledge about the GOP structure.
  //       Using getIntraPeriod() should be avoided though, because it assumes certain GOP
  //       properties, which are only valid in CTC.
  //       Still copying this setting from HM for consistency, improvements welcome
  bool isRandomAccess  = m_pcCfg->getIntraPeriod() > 0;
  if( isRandomAccess )
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 6;                        // 32 = 2^5 (plus 1)
    bufferingPeriodSEI->m_dpbOutputDelayLength =  6;                        // 32 + 3 = 2^6
  }
  else
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 9;                        // max. 2^10
    bufferingPeriodSEI->m_dpbOutputDelayLength =  9;                        // max. 2^10
  }
  bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength = 7;               // ceil( log2( tick_divisor_minus2 + 2 ) )
  bufferingPeriodSEI->m_dpbOutputDelayDuLength = bufferingPeriodSEI->m_dpbOutputDelayLength + bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength;
  //for the concatenation, it can be set to one during splicing.
  bufferingPeriodSEI->m_concatenationFlag = 0;
  //since the temporal layer HRDParameters is not ready, we assumed it is fixed
  bufferingPeriodSEI->m_auCpbRemovalDelayDelta = 1;
  bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag = m_pcCfg->getBpDeltasGOPStructure() ;
  if (bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag)
  {
    switch (m_pcCfg->getGOPSize())
    {
      case 8:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 7;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
        }
      }
        break;
      case 16:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 4;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[5]          = 7;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[6]          = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[7]          = 14;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[8]          = 15;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 7;
        }
      }
        break;
      default:
      {
        THROW("m_cpbRemovalDelayDelta not applicable for the GOP size");
      }
        break;
    }
  }
  bufferingPeriodSEI->m_sublayerDpbOutputOffsetsPresentFlag = true;
  for(int i = 0; i < bufferingPeriodSEI->m_bpMaxSubLayers; i++)
  {
    bufferingPeriodSEI->m_dpbOutputTidOffset[i] = m_pcCfg->getMaxNumReorderPics(i) * static_cast<int>(pow(2, static_cast<double>(bufferingPeriodSEI->m_bpMaxSubLayers-1-i)));
    if(bufferingPeriodSEI->m_dpbOutputTidOffset[i] >= m_pcCfg->getMaxNumReorderPics(bufferingPeriodSEI->m_bpMaxSubLayers-1))
    {
      bufferingPeriodSEI->m_dpbOutputTidOffset[i] -= m_pcCfg->getMaxNumReorderPics(bufferingPeriodSEI->m_bpMaxSubLayers-1);
    }
    else
    {
      bufferingPeriodSEI->m_dpbOutputTidOffset[i] = 0;
    }
  }
  // A commercial encoder should track the buffer state for all layers and sub-layers
  // to ensure CPB conformance. Such tracking is required for calculating alternative
  // CPB parameters.
  // Unfortunately VTM does not have such tracking. Thus we cannot encode alternative
  // CPB parameters here.
  bufferingPeriodSEI->m_altCpbParamsPresentFlag = false;
  bufferingPeriodSEI->m_useAltCpbParamsFlag = false;
}

void SEIEncoder::initSEIErp(SEIEquirectangularProjection* seiEquirectangularProjection)
{
  CHECK(!(m_isInitialized), "seiEquirectangularProjection already initialized");
  CHECK(!(seiEquirectangularProjection != nullptr), "Need a seiEquirectangularProjection for initialization (got nullptr)");

  seiEquirectangularProjection->m_erpCancelFlag = m_pcCfg->getErpSEICancelFlag();
  if (!seiEquirectangularProjection->m_erpCancelFlag)
  {
    seiEquirectangularProjection->m_erpPersistenceFlag   = m_pcCfg->getErpSEIPersistenceFlag();
    seiEquirectangularProjection->m_erpGuardBandFlag     = m_pcCfg->getErpSEIGuardBandFlag();
    if (seiEquirectangularProjection->m_erpGuardBandFlag == 1)
    {
      seiEquirectangularProjection->m_erpGuardBandType       = m_pcCfg->getErpSEIGuardBandType();
      seiEquirectangularProjection->m_erpLeftGuardBandWidth  = m_pcCfg->getErpSEILeftGuardBandWidth();
      seiEquirectangularProjection->m_erpRightGuardBandWidth = m_pcCfg->getErpSEIRightGuardBandWidth();
    }
  }
}

#if GREEN_METADATA_SEI_ENABLED
void SEIEncoder::initSEIGreenMetadataInfo(SEIGreenMetadataInfo* seiGreenMetadataInfo, FeatureCounterStruct featureCounter, SEIQualityMetrics metrics,SEIComplexityMetrics greenMetadata)
{
  assert (m_isInitialized);
  assert (seiGreenMetadataInfo!=NULL);
  
  if (m_pcCfg->getSEIGreenMetadataType() == 1) //Metadata for quality recovery after low-power encoding
  {
    seiGreenMetadataInfo->m_greenMetadataType = m_pcCfg->getSEIGreenMetadataType();
    seiGreenMetadataInfo->m_xsdSubpicNumberMinus1 = m_pcCfg->getSEIXSDNumberMetrics()-1;
    seiGreenMetadataInfo->m_xsdSubPicIdc = 1; //Only 1 Picture is supported
    // Maximum valid value for 16-bit integer: 65535
    (m_pcCfg->getSEIXSDMetricTypePSNR())
      ? seiGreenMetadataInfo->m_xsdMetricValuePSNR = std::min(int(metrics.psnr * 100), 65535)
      : seiGreenMetadataInfo->m_xsdMetricValuePSNR = 0;
    (m_pcCfg->getSEIXSDMetricTypeSSIM())
      ? seiGreenMetadataInfo->m_xsdMetricValueSSIM = std::min(int(metrics.ssim * 100), 65535)
      : seiGreenMetadataInfo->m_xsdMetricValueSSIM = 0;
    (m_pcCfg->getSEIXSDMetricTypeWPSNR())
      ? seiGreenMetadataInfo->m_xsdMetricValueWPSNR = std::min(int(metrics.wpsnr * 100), 65535)
      : seiGreenMetadataInfo->m_xsdMetricValueWPSNR = 0;
    (m_pcCfg->getSEIXSDMetricTypeWSPSNR())
      ? seiGreenMetadataInfo->m_xsdMetricValueWSPSNR = std::min(int(metrics.wspsnr * 100), 65535)
      : seiGreenMetadataInfo->m_xsdMetricValueWSPSNR = 0;

    seiGreenMetadataInfo->m_xsdMetricTypePSNR = m_pcCfg->getSEIXSDMetricTypePSNR();
    seiGreenMetadataInfo->m_xsdMetricTypeSSIM = m_pcCfg->getSEIXSDMetricTypeSSIM();
    seiGreenMetadataInfo->m_xsdMetricTypeWPSNR = m_pcCfg->getSEIXSDMetricTypeWPSNR();
    seiGreenMetadataInfo->m_xsdMetricTypeWSPSNR = m_pcCfg->getSEIXSDMetricTypeWSPSNR();
  }
  else if(m_pcCfg->getSEIGreenMetadataType() == 0) // Metadata for decoder-complexity metrics
  {
    seiGreenMetadataInfo->m_greenMetadataType                   = m_pcCfg->getSEIGreenMetadataType();
    seiGreenMetadataInfo->m_greenMetadataGranularityType        = m_pcCfg->getSEIGreenMetadataGranularityType();
    seiGreenMetadataInfo->m_greenMetadataExtendedRepresentation = m_pcCfg->getSEIGreenMetadataExtendedRepresentation();
    switch (m_pcCfg->getSEIGreenMetadataPeriodType())   // Period type
    {
    case 0:   // 0x00 complexity metrics are applicable to a single picture
      seiGreenMetadataInfo->m_numPictures = m_pcCfg->getSEIGreenMetadataPeriodNumPictures();
      break;
    case 1:   // 0x01 complexity metrics are applicable to all pictures in decoding order, up to (but not including) the picture containing the next I slice
      //
      break;
    case 2:   // 0x02 complexity metrics are applicable over a specified time interval in seconds
      seiGreenMetadataInfo->m_numPictures = m_pcCfg->getSEIGreenMetadataPeriodNumPictures();
      break;
    case 3:   // 0x03 complexity metrics are applicable over a specified number of pictures counted in decoding order
      seiGreenMetadataInfo->m_numSeconds = m_pcCfg->getSEIGreenMetadataPeriodNumSeconds();
      break;
    default:   // 0x05-0xFF reserved
      break;   //
    }
  }
}
#endif

void SEIEncoder::initSEISphereRotation(SEISphereRotation* seiSphereRotation)
{
  CHECK(!(m_isInitialized), "seiSphereRotation already initialized");
  CHECK(!(seiSphereRotation != nullptr), "Need a seiSphereRotation for initialization (got nullptr)");

  seiSphereRotation->m_sphereRotationCancelFlag = m_pcCfg->getSphereRotationSEICancelFlag();
  if ( !seiSphereRotation->m_sphereRotationCancelFlag )
  {
    seiSphereRotation->m_sphereRotationPersistenceFlag = m_pcCfg->getSphereRotationSEIPersistenceFlag();
    seiSphereRotation->m_sphereRotationYaw = m_pcCfg->getSphereRotationSEIYaw();
    seiSphereRotation->m_sphereRotationPitch = m_pcCfg->getSphereRotationSEIPitch();
    seiSphereRotation->m_sphereRotationRoll = m_pcCfg->getSphereRotationSEIRoll();
  }
}

void SEIEncoder::initSEIOmniViewport(SEIOmniViewport* seiOmniViewport)
{
  CHECK(!(m_isInitialized), "seiOmniViewport already initialized");
  CHECK(!(seiOmniViewport != nullptr), "Need a seiOmniViewport for initialization (got nullptr)");

  seiOmniViewport->m_omniViewportId = m_pcCfg->getOmniViewportSEIId();
  seiOmniViewport->m_omniViewportCancelFlag = m_pcCfg->getOmniViewportSEICancelFlag();
  if ( !seiOmniViewport->m_omniViewportCancelFlag )
  {
    seiOmniViewport->m_omniViewportPersistenceFlag = m_pcCfg->getOmniViewportSEIPersistenceFlag();
    seiOmniViewport->m_omniViewportCntMinus1 = m_pcCfg->getOmniViewportSEICntMinus1();

    seiOmniViewport->m_omniViewportRegions.resize(seiOmniViewport->m_omniViewportCntMinus1+1);
    for (uint32_t i = 0; i <= seiOmniViewport->m_omniViewportCntMinus1; i++)
    {
      SEIOmniViewport::OmniViewport &viewport = seiOmniViewport->m_omniViewportRegions[i];
      viewport.azimuthCentre   = m_pcCfg->getOmniViewportSEIAzimuthCentre(i);
      viewport.elevationCentre = m_pcCfg->getOmniViewportSEIElevationCentre(i);
      viewport.tiltCentre      = m_pcCfg->getOmniViewportSEITiltCentre(i);
      viewport.horRange        = m_pcCfg->getOmniViewportSEIHorRange(i);
      viewport.verRange        = m_pcCfg->getOmniViewportSEIVerRange(i);
    }
  }
}

void SEIEncoder::initSEIRegionWisePacking(SEIRegionWisePacking *seiRegionWisePacking)
{
  CHECK(!(m_isInitialized), "seiRegionWisePacking already initialized");
  CHECK(!(seiRegionWisePacking != nullptr), "Need a seiRegionWisePacking for initialization (got nullptr)");

  seiRegionWisePacking->m_rwpCancelFlag                          = m_pcCfg->getRwpSEIRwpCancelFlag();
  seiRegionWisePacking->m_rwpPersistenceFlag                     = m_pcCfg->getRwpSEIRwpPersistenceFlag();
  seiRegionWisePacking->m_constituentPictureMatchingFlag         = m_pcCfg->getRwpSEIConstituentPictureMatchingFlag();
  seiRegionWisePacking->m_numPackedRegions                       = m_pcCfg->getRwpSEINumPackedRegions();
  seiRegionWisePacking->m_projPictureWidth                       = m_pcCfg->getRwpSEIProjPictureWidth();
  seiRegionWisePacking->m_projPictureHeight                      = m_pcCfg->getRwpSEIProjPictureHeight();
  seiRegionWisePacking->m_packedPictureWidth                     = m_pcCfg->getRwpSEIPackedPictureWidth();
  seiRegionWisePacking->m_packedPictureHeight                    = m_pcCfg->getRwpSEIPackedPictureHeight();
  seiRegionWisePacking->m_rwpTransformType.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpProjRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpLeftGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpRightGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpTopGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpBottomGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandType.resize(4*seiRegionWisePacking->m_numPackedRegions);
  for( int i=0; i < seiRegionWisePacking->m_numPackedRegions; i++ )
  {
    seiRegionWisePacking->m_rwpTransformType[i]                  = m_pcCfg->getRwpSEIRwpTransformType(i);
    seiRegionWisePacking->m_rwpGuardBandFlag[i]                  = m_pcCfg->getRwpSEIRwpGuardBandFlag(i);
    seiRegionWisePacking->m_projRegionWidth[i]                   = m_pcCfg->getRwpSEIProjRegionWidth(i);
    seiRegionWisePacking->m_projRegionHeight[i]                  = m_pcCfg->getRwpSEIProjRegionHeight(i);
    seiRegionWisePacking->m_rwpProjRegionTop[i]                  = m_pcCfg->getRwpSEIRwpSEIProjRegionTop(i);
    seiRegionWisePacking->m_projRegionLeft[i]                    = m_pcCfg->getRwpSEIProjRegionLeft(i);
    seiRegionWisePacking->m_packedRegionWidth[i]                 = m_pcCfg->getRwpSEIPackedRegionWidth(i);
    seiRegionWisePacking->m_packedRegionHeight[i]                = m_pcCfg->getRwpSEIPackedRegionHeight(i);
    seiRegionWisePacking->m_packedRegionTop[i]                   = m_pcCfg->getRwpSEIPackedRegionTop(i);
    seiRegionWisePacking->m_packedRegionLeft[i]                  = m_pcCfg->getRwpSEIPackedRegionLeft(i);
    if( seiRegionWisePacking->m_rwpGuardBandFlag[i] )
    {
      seiRegionWisePacking->m_rwpLeftGuardBandWidth[i]           =  m_pcCfg->getRwpSEIRwpLeftGuardBandWidth(i);
      seiRegionWisePacking->m_rwpRightGuardBandWidth[i]          =  m_pcCfg->getRwpSEIRwpRightGuardBandWidth(i);
      seiRegionWisePacking->m_rwpTopGuardBandHeight[i]           =  m_pcCfg->getRwpSEIRwpTopGuardBandHeight(i);
      seiRegionWisePacking->m_rwpBottomGuardBandHeight[i]        =  m_pcCfg->getRwpSEIRwpBottomGuardBandHeight(i);
      seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag[i]  =  m_pcCfg->getRwpSEIRwpGuardBandNotUsedForPredFlag(i);
      for( int j=0; j < 4; j++ )
      {
        seiRegionWisePacking->m_rwpGuardBandType[i*4 + j]         =  m_pcCfg->getRwpSEIRwpGuardBandType(i*4 + j);
      }
    }
  }
}

void SEIEncoder::initSEIGcmp(SEIGeneralizedCubemapProjection* seiGeneralizedCubemapProjection)
{
  CHECK(!(m_isInitialized), "seiGeneralizedCubemapProjection already initialized");
  CHECK(!(seiGeneralizedCubemapProjection != nullptr), "Need a seiGeneralizedCubemapProjection for initialization (got nullptr)");

  seiGeneralizedCubemapProjection->m_gcmpCancelFlag                      = m_pcCfg->getGcmpSEICancelFlag();
  if (!seiGeneralizedCubemapProjection->m_gcmpCancelFlag)
  {
    seiGeneralizedCubemapProjection->m_gcmpPersistenceFlag               = m_pcCfg->getGcmpSEIPersistenceFlag();
    seiGeneralizedCubemapProjection->m_gcmpPackingType                   = m_pcCfg->getGcmpSEIPackingType();
    seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType           = m_pcCfg->getGcmpSEIMappingFunctionType();

    int numFace = seiGeneralizedCubemapProjection->m_gcmpPackingType == 4 || seiGeneralizedCubemapProjection->m_gcmpPackingType == 5 ? 5 : 6;
    seiGeneralizedCubemapProjection->m_gcmpFaceIndex.resize(numFace);
    seiGeneralizedCubemapProjection->m_gcmpFaceRotation.resize(numFace);
    if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
    {
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag.resize(numFace);
    }
    for (int i = 0; i < numFace; i++)
    {
      seiGeneralizedCubemapProjection->m_gcmpFaceIndex[i]                = m_pcCfg->getGcmpSEIFaceIndex(i);
      seiGeneralizedCubemapProjection->m_gcmpFaceRotation[i]             = m_pcCfg->getGcmpSEIFaceRotation(i);
      if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
      {
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU[i]           = std::max<uint8_t>(1, (uint8_t)(128.0 * m_pcCfg->getGcmpSEIFunctionCoeffU(i) + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag[i] = m_pcCfg->getGcmpSEIFunctionUAffectedByVFlag(i);
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV[i]           = std::max<uint8_t>(1, (uint8_t)(128.0 * m_pcCfg->getGcmpSEIFunctionCoeffV(i) + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag[i] = m_pcCfg->getGcmpSEIFunctionVAffectedByUFlag(i);
      }
    }

    seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag                 = m_pcCfg->getGcmpSEIGuardBandFlag();
    if (seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag)
    {
      seiGeneralizedCubemapProjection->m_gcmpGuardBandType                 = m_pcCfg->getGcmpSEIGuardBandType();
      seiGeneralizedCubemapProjection->m_gcmpGuardBandBoundaryExteriorFlag = m_pcCfg->getGcmpSEIGuardBandBoundaryExteriorFlag();
      seiGeneralizedCubemapProjection->m_gcmpGuardBandSamplesMinus1        = m_pcCfg->getGcmpSEIGuardBandSamplesMinus1();
    }
  }
}

void SEIEncoder::initSEISampleAspectRatioInfo(SEISampleAspectRatioInfo* seiSampleAspectRatioInfo)
{
  CHECK(!(m_isInitialized), "seiSampleAspectRatioInfo already initialized");
  CHECK(!(seiSampleAspectRatioInfo != nullptr), "Need a seiSampleAspectRatioInfo for initialization (got nullptr)");

  seiSampleAspectRatioInfo->m_sariCancelFlag = m_pcCfg->getSariCancelFlag();
  if (!seiSampleAspectRatioInfo->m_sariCancelFlag)
  {
    seiSampleAspectRatioInfo->m_sariPersistenceFlag   = m_pcCfg->getSariPersistenceFlag();
    seiSampleAspectRatioInfo->m_sariAspectRatioIdc    = m_pcCfg->getSariAspectRatioIdc();
    if (seiSampleAspectRatioInfo->m_sariAspectRatioIdc == 255)
    {
      seiSampleAspectRatioInfo->m_sariSarWidth   = m_pcCfg->getSariSarWidth();
      seiSampleAspectRatioInfo->m_sariSarHeight  = m_pcCfg->getSariSarHeight();
    }
    else
    {
      seiSampleAspectRatioInfo->m_sariSarWidth   = 0;
      seiSampleAspectRatioInfo->m_sariSarHeight  = 0;
    }
  }
}

void SEIEncoder::initSEIPhaseIndication(SEIPhaseIndication* seiPhaseIndication, int ppsId)
{
  CHECK(!(m_isInitialized), "seiPhaseIndication already initialized");
  CHECK(!(seiPhaseIndication != nullptr), "Need a seiPhaseIndication for initialization (got nullptr)");

  if (ppsId == 0)
  {
    seiPhaseIndication->m_horPhaseNum = m_pcCfg->getHorPhaseNumFullResolution();
    seiPhaseIndication->m_horPhaseDenMinus1 = m_pcCfg->getHorPhaseDenMinus1FullResolution();
    seiPhaseIndication->m_verPhaseNum = m_pcCfg->getVerPhaseNumFullResolution();
    seiPhaseIndication->m_verPhaseDenMinus1 = m_pcCfg->getVerPhaseDenMinus1FullResolution();
  }
  else if (ppsId == ENC_PPS_ID_RPR)
  {
    seiPhaseIndication->m_horPhaseNum = m_pcCfg->getHorPhaseNumReducedResolution();
    seiPhaseIndication->m_horPhaseDenMinus1 = m_pcCfg->getHorPhaseDenMinus1ReducedResolution();
    seiPhaseIndication->m_verPhaseNum = m_pcCfg->getVerPhaseNumReducedResolution();
    seiPhaseIndication->m_verPhaseDenMinus1 = m_pcCfg->getVerPhaseDenMinus1ReducedResolution();
  }
}

//! initialize scalable nesting SEI message.
//! Note: The SEI message structures input into this function will become part of the scalable nesting SEI and will be
//!       automatically freed, when the nesting SEI is disposed.
//  either targetOLS or targetLayer should be active, call with empty vector for the inactive mode
void SEIEncoder::initSEIScalableNesting(SEIScalableNesting *scalableNestingSEI, SEIMessages &nestedSEIs, const std::vector<int> &targetOLSs, const std::vector<int> &targetLayers, const std::vector<uint16_t> &subpictureIDs, uint16_t maxSubpicIdInPic)
{
  CHECK(!(m_isInitialized), "Scalable Nesting SEI already initialized ");
  CHECK(!(scalableNestingSEI != nullptr), "No Scalable Nesting SEI object passed");
  CHECK (targetOLSs.size() > 0 && targetLayers.size() > 0, "Scalable Nesting SEI can apply to either OLS or layer(s), not both");

  scalableNestingSEI->m_snOlsFlag = (targetOLSs.size() > 0) ? 1 : 0;  // If the nested SEI messages are picture buffering SEI messages, picture timing SEI messages or sub-picture timing SEI messages, nesting_ols_flag shall be equal to 1, by default case
  if (scalableNestingSEI->m_snOlsFlag)
  {
    scalableNestingSEI->m_snNumOlssMinus1 =  (uint32_t) targetOLSs.size() - 1;
    // initialize absolute indexes
    for (int i = 0; i <= scalableNestingSEI->m_snNumOlssMinus1; i++)
    {
      scalableNestingSEI->m_snOlsIdx[i] = targetOLSs[i];
    }
    // calculate delta indexes from absolute ones
    for (int i = 0; i <= scalableNestingSEI->m_snNumOlssMinus1; i++)
    {
      if (i == 0)
      {
        CHECK (scalableNestingSEI->m_snOlsIdx[i] < 0, "OLS indexes must be  equal to or greater than 0");
        // no "-1" operation for the first index although the name implies one
        scalableNestingSEI->m_snOlsIdxDeltaMinus1[i] = scalableNestingSEI->m_snOlsIdx[i];
      }
      else
      {
        CHECK (scalableNestingSEI->m_snOlsIdx[i] <= scalableNestingSEI->m_snOlsIdx[i - 1], "OLS indexes must be in ascending order");
        scalableNestingSEI->m_snOlsIdxDeltaMinus1[i] = scalableNestingSEI->m_snOlsIdx[i] - scalableNestingSEI->m_snOlsIdx[i - 1] - 1;
      }
    }
  }
  else
  {
    scalableNestingSEI->m_snAllLayersFlag = 0;                          // nesting is not applied to all layers
    scalableNestingSEI->m_snNumLayersMinus1 = (uint32_t) targetLayers.size() - 1;  //nesting_num_layers_minus1
    for (int i=0; i <= scalableNestingSEI->m_snNumLayersMinus1; i++ )
    {
      scalableNestingSEI->m_snLayerId[i] = targetLayers[i];
    }
  }
  if (!subpictureIDs.empty())
  {
    scalableNestingSEI->m_snSubpicFlag = 1;
    scalableNestingSEI->m_snNumSubpics = (uint32_t) subpictureIDs.size();
    scalableNestingSEI->m_snSubpicId   = subpictureIDs;
    scalableNestingSEI->m_snSubpicIdLen = std::max(1, ceilLog2(maxSubpicIdInPic + 1));
    CHECK ( scalableNestingSEI->m_snSubpicIdLen > 16, "Subpicture ID too large. Length must be <= 16 bits");
  }
  scalableNestingSEI->m_nestedSEIs.clear();
  for (SEIMessages::iterator it = nestedSEIs.begin(); it != nestedSEIs.end(); it++)
  {
    scalableNestingSEI->m_nestedSEIs.push_back((*it));
  }
}


//! calculate hashes for entire reconstructed picture
void SEIEncoder::initDecodedPictureHashSEI(SEIDecodedPictureHash *decodedPictureHashSEI, PelUnitBuf& pic, std::string &rHashString, const BitDepths &bitDepths)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(decodedPictureHashSEI != nullptr), "Unspecified error");

  decodedPictureHashSEI->method = m_pcCfg->getDecodedPictureHashSEIType();
  decodedPictureHashSEI->singleCompFlag = !isChromaEnabled(m_pcCfg->getChromaFormatIdc());
  switch (m_pcCfg->getDecodedPictureHashSEIType())
  {
  case HashType::MD5:
  {
    uint32_t numChar = calcMD5(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
    rHashString      = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
    break;
  }
  break;
  case HashType::CRC:
  {
    uint32_t numChar = calcCRC(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
    rHashString      = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
    break;
  }
  case HashType::CHECKSUM:
  default:
  {
    uint32_t numChar = calcChecksum(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
    rHashString      = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
    break;
  }
  }
}

void SEIEncoder::initSEIDependentRAPIndication(SEIDependentRAPIndication *seiDependentRAPIndication)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiDependentRAPIndication != nullptr), "Unspecified error");
}

void SEIEncoder::initSEIExtendedDrapIndication(SEIExtendedDrapIndication *sei)
{
  CHECK(!(m_isInitialized), "Extended DRAP SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiExtendedDrapIndication for initialization (got nullptr)");
  sei->m_edrapIndicationRapIdMinus1 = 0;
  sei->m_edrapIndicationLeadingPicturesDecodableFlag = false;
  sei->m_edrapIndicationReservedZero12Bits = 0;
  sei->m_edrapIndicationNumRefRapPicsMinus1 = 0;
  sei->m_edrapIndicationRefRapId.resize(sei->m_edrapIndicationNumRefRapPicsMinus1 + 1);
  for (int i = 0; i <= sei->m_edrapIndicationNumRefRapPicsMinus1; i++)
  {
    sei->m_edrapIndicationRefRapId[i] = 0;
  }
}

void SEIEncoder::initSEIShutterIntervalInfo(SEIShutterIntervalInfo *seiShutterIntervalInfo)
{
  assert(m_isInitialized);
  assert(seiShutterIntervalInfo != nullptr);
  seiShutterIntervalInfo->m_siiTimeScale = m_pcCfg->getSiiSEITimeScale();
  seiShutterIntervalInfo->m_siiFixedSIwithinCLVS = m_pcCfg->getSiiSEIFixedSIwithinCLVS();
  if (seiShutterIntervalInfo->m_siiFixedSIwithinCLVS == true)
  {
    seiShutterIntervalInfo->m_siiNumUnitsInShutterInterval = m_pcCfg->getSiiSEINumUnitsInShutterInterval();
  }
  else
  {
    seiShutterIntervalInfo->m_siiMaxSubLayersMinus1 = m_pcCfg->getSiiSEIMaxSubLayersMinus1();
    seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI.resize(seiShutterIntervalInfo->m_siiMaxSubLayersMinus1 + 1);
    for (int32_t i = 0; i <= seiShutterIntervalInfo->m_siiMaxSubLayersMinus1; i++)
    {
      seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI[i] = m_pcCfg->getSiiSEISubLayerNumUnitsInSI(i);
    }
  }
}

#if JVET_AF0310_PO_NESTING
void SEIEncoder::initSEIProcessingOrderInfo(SEIProcessingOrderInfo *seiProcessingOrderInfo, SEIProcessingOrderNesting *seiProcessingOrderNesting)
#else
void SEIEncoder::initSEIProcessingOrderInfo(SEIProcessingOrderInfo *seiProcessingOrderInfo)
#endif
{
  assert(m_isInitialized);
  assert(seiProcessingOrderInfo != nullptr);


  seiProcessingOrderInfo->m_posEnabled          = m_pcCfg->getPoSEIEnabled();
#if JVET_AF0061_ADDITION_PO_ID
  seiProcessingOrderInfo->m_posId               = m_pcCfg->getPoSEIId();
#endif
  seiProcessingOrderInfo->m_posNumMinus2        = m_pcCfg->getPoSEINumMinus2();
  seiProcessingOrderInfo->m_posWrappingFlag.resize(m_pcCfg->getPoSEIPayloadTypeSize());
  seiProcessingOrderInfo->m_posImportanceFlag.resize(m_pcCfg->getPoSEIPayloadTypeSize());
#if !JVET_AF0310_PO_NESTING
  seiProcessingOrderInfo->m_posWrapSeiMessages.clear();
#endif
  seiProcessingOrderInfo->m_posPrefixFlag.resize(m_pcCfg->getPoSEIPayloadTypeSize());
  seiProcessingOrderInfo->m_posPayloadType.resize(m_pcCfg->getPoSEIPayloadTypeSize());
  seiProcessingOrderInfo->m_posProcessingOrder.resize(m_pcCfg->getPoSEIPayloadTypeSize());
#if JVET_AF0310_PO_NESTING
  seiProcessingOrderInfo->m_posNumBitsInPrefix.resize(m_pcCfg->getPoSEIPayloadTypeSize());
#endif
  seiProcessingOrderInfo->m_posPrefixByte.resize(m_pcCfg->getPoSEIPayloadTypeSize());
  for (uint32_t i = 0; i < (m_pcCfg->getPoSEINumMinus2() + 2); i++)
  {
    seiProcessingOrderInfo->m_posWrappingFlag[i] = m_pcCfg->getPoSEIWrappingFlag(i);
    seiProcessingOrderInfo->m_posImportanceFlag[i] = m_pcCfg->getPoSEIImportanceFlag(i);
    seiProcessingOrderInfo->m_posPrefixFlag[i] = m_pcCfg->getPoSEIPrefixFlag(i);
    seiProcessingOrderInfo->m_posPayloadType[i]     = m_pcCfg->getPoSEIPayloadType(i);
    seiProcessingOrderInfo->m_posProcessingOrder[i] = m_pcCfg->getPoSEIProcessingOrder(i);
#if JVET_AF0310_PO_NESTING
    seiProcessingOrderInfo->m_posNumBitsInPrefix[i] = m_pcCfg->getPoSEINumOfPrefixBits(i);
#endif
    if (seiProcessingOrderInfo->m_posPrefixFlag[i])
    {
      seiProcessingOrderInfo->m_posPrefixByte[i] = m_pcCfg->getPoSEIPrefixByte(i);
    }
  }
#if JVET_AF0310_PO_NESTING
  seiProcessingOrderNesting->m_ponTargetPoId.clear();
  seiProcessingOrderNesting->m_ponPayloadType.clear();
  seiProcessingOrderNesting->m_ponProcessingOrder.clear();
  seiProcessingOrderNesting->m_ponWrapSeiMessages.clear();
  seiProcessingOrderNesting->m_ponTargetPoId.push_back((uint8_t)seiProcessingOrderInfo->m_posId);
  uint32_t ponNumSeis = 0;
#endif
  for (uint32_t i = 0; i < (m_pcCfg->getPoSEINumMinus2() + 2); i++)
  {
    if (seiProcessingOrderInfo->m_posWrappingFlag[i])
    {
      CHECK(!seiProcessingOrderInfo->checkWrappingSEIPayloadType(SEI::PayloadType(seiProcessingOrderInfo->m_posPayloadType[i])), "not support in sei processing order SEI");
#if JVET_AF0310_PO_NESTING
      seiProcessingOrderNesting->m_ponPayloadType.push_back(seiProcessingOrderInfo->m_posPayloadType[i]);
      seiProcessingOrderNesting->m_ponProcessingOrder.push_back((uint8_t)seiProcessingOrderInfo->m_posProcessingOrder[i]);
      ponNumSeis++;
#endif
      switch (SEI::PayloadType(seiProcessingOrderInfo->m_posPayloadType[i]))
      {
      case SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS:
      {
        SEIFilmGrainCharacteristics* seiFGC = new SEIFilmGrainCharacteristics;
        initSEIFilmGrainCharacteristics(seiFGC);
#if JVET_AF0310_PO_NESTING
        seiProcessingOrderNesting->m_ponWrapSeiMessages.push_back(seiFGC);
#else
        seiProcessingOrderInfo->m_posWrapSeiMessages.push_back(seiFGC);
#endif
        break;
      }
      case SEI::PayloadType::CONTENT_LIGHT_LEVEL_INFO:
      {
        SEIContentLightLevelInfo* seiCCL = new SEIContentLightLevelInfo;
        initSEIContentLightLevel(seiCCL);
#if JVET_AF0310_PO_NESTING
        seiProcessingOrderNesting->m_ponWrapSeiMessages.push_back(seiCCL);
#else
        seiProcessingOrderInfo->m_posWrapSeiMessages.push_back(seiCCL);
#endif
        break;
      }
      case SEI::PayloadType::CONTENT_COLOUR_VOLUME:
      {
        SEIContentColourVolume* seiCCV = new SEIContentColourVolume;
        initSEIContentColourVolume(seiCCV);
#if JVET_AF0310_PO_NESTING
        seiProcessingOrderNesting->m_ponWrapSeiMessages.push_back(seiCCV);
#else
        seiProcessingOrderInfo->m_posWrapSeiMessages.push_back(seiCCV);
#endif
        break;
      }
      case SEI::PayloadType::COLOUR_TRANSFORM_INFO:
      {
        SEIColourTransformInfo* seiCTI = new SEIColourTransformInfo;
        initSEIColourTransformInfo(seiCTI);
#if JVET_AF0310_PO_NESTING
        seiProcessingOrderNesting->m_ponWrapSeiMessages.push_back(seiCTI);
#else
        seiProcessingOrderInfo->m_posWrapSeiMessages.push_back(seiCTI);
#endif
        break;
      }
      case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS:
      {
        SEINeuralNetworkPostFilterCharacteristics* seiNNPFC = new  SEINeuralNetworkPostFilterCharacteristics;
        initSEINeuralNetworkPostFilterCharacteristics(seiNNPFC, 0);
#if JVET_AF0310_PO_NESTING
        seiProcessingOrderNesting->m_ponWrapSeiMessages.push_back(seiNNPFC);
#else
        seiProcessingOrderInfo->m_posWrapSeiMessages.push_back(seiNNPFC);
#endif
        break;
      }
      case SEI::PayloadType::POST_FILTER_HINT:
      {
        SEIPostFilterHint* seiPFH = new SEIPostFilterHint;
        initSEIPostFilterHint(seiPFH);
#if JVET_AF0310_PO_NESTING
        seiProcessingOrderNesting->m_ponWrapSeiMessages.push_back(seiPFH);
#else
        seiProcessingOrderInfo->m_posWrapSeiMessages.push_back(seiPFH);
#endif
        break;
      }
      default:
      {
        msg(ERROR, "not support in sei processing order SEI\n");
        exit(1);
      }
      }
    }
  }
#if JVET_AF0310_PO_NESTING
  CHECK(ponNumSeis == 0, "Number of PO nested SEI messages must be greater than 0 ");
  seiProcessingOrderNesting->m_ponNumSeisMinus1 = ponNumSeis - 1;
#endif
}

void SEIEncoder::initSEIPostFilterHint(SEIPostFilterHint *seiPostFilterHint)
{
  CHECK(!m_isInitialized, "The post-filter hint SEI message needs to be initialized");
  CHECK(seiPostFilterHint == nullptr, "Failed to get the handler to the SEI message");

  seiPostFilterHint->m_filterHintCancelFlag             = m_pcCfg->getPostFilterHintSEICancelFlag();
  seiPostFilterHint->m_filterHintPersistenceFlag        = m_pcCfg->getPostFilterHintSEIPersistenceFlag();
  seiPostFilterHint->m_filterHintSizeY                  = m_pcCfg->getPostFilterHintSEISizeY();
  seiPostFilterHint->m_filterHintSizeX                  = m_pcCfg->getPostFilterHintSEISizeX();
  seiPostFilterHint->m_filterHintType                   = m_pcCfg->getPostFilterHintSEIType();
  seiPostFilterHint->m_filterHintChromaCoeffPresentFlag = m_pcCfg->getPostFilterHintSEIChromaCoeffPresentFlag();

  seiPostFilterHint->m_filterHintValues.resize((seiPostFilterHint->m_filterHintChromaCoeffPresentFlag ? 3 : 1)
                                               * seiPostFilterHint->m_filterHintSizeY
                                               * seiPostFilterHint->m_filterHintSizeX);
  for (uint32_t i = 0; i < seiPostFilterHint->m_filterHintValues.size(); i++)
  {
    seiPostFilterHint->m_filterHintValues[i] = m_pcCfg->getPostFilterHintSEIValues(i);
  }
}

template <typename T>
static void readTokenValue(T            &returnedValue, /// value returned
                           bool         &failed,        /// used and updated
                           std::istream &is,            /// stream to read token from
                           const char  *pToken)        /// token string
{
  returnedValue=T();
  if (failed)
  {
    return;
  }

  int c;
  // Ignore any whitespace
  while ((c=is.get())!=EOF && isspace(c));
  // test for comment mark
  while (c=='#')
  {
    // Ignore to the end of the line
    while ((c=is.get())!=EOF && (c!=10 && c!=13));
    // Ignore any white space at the start of the next line
    while ((c=is.get())!=EOF && isspace(c));
  }
  // test first character of token
  failed=(c!=pToken[0]);
  // test remaining characters of token
  int pos;
  for(pos=1;!failed && pToken[pos]!=0 && is.get()==pToken[pos]; pos++);
  failed|=(pToken[pos]!=0);
  // Ignore any whitespace before the ':'
  while (!failed && (c=is.get())!=EOF && isspace(c));
  failed|=(c!=':');
  // Now read the value associated with the token:
  if (!failed)
  {
    is >> returnedValue;
    failed=!is.good();
    if (!failed)
    {
      c=is.get();
      failed=(c!=EOF && !isspace(c));
    }
  }
  if (failed)
  {
    std::cerr << "Unable to read token '" << pToken << "'\n";
  }
}

template <typename T>
static void readTokenValueAndValidate(T            &returnedValue, /// value returned
                                      bool         &failed,        /// used and updated
                                      std::istream &is,            /// stream to read token from
                                      const char  *pToken,        /// token string
                                      const T      &minInclusive,  /// minimum value allowed, inclusive
                                      const T      &maxInclusive)  /// maximum value allowed, inclusive
{
  readTokenValue(returnedValue, failed, is, pToken);
  if (!failed)
  {
    if (returnedValue<minInclusive || returnedValue>maxInclusive)
    {
      failed=true;
      std::cerr << "Value for token " << pToken << " must be in the range " << minInclusive << " to " << maxInclusive << " (inclusive); value read: " << returnedValue << std::endl;
    }
  }
}

void SEIEncoder::readAnnotatedRegionSEI(std::istream &fic, SEIAnnotatedRegions *seiAnnoRegion, bool &failed)
{
  readTokenValue(seiAnnoRegion->m_hdr.m_cancelFlag, failed, fic, "SEIArCancelFlag");
  if (!seiAnnoRegion->m_hdr.m_cancelFlag)
  {
    readTokenValue(seiAnnoRegion->m_hdr.m_notOptimizedForViewingFlag, failed, fic, "SEIArNotOptForViewingFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_trueMotionFlag, failed, fic, "SEIArTrueMotionFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_occludedObjectFlag, failed, fic, "SEIArOccludedObjsFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_partialObjectFlagPresentFlag, failed, fic, "SEIArPartialObjsFlagPresentFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_objectLabelPresentFlag, failed, fic, "SEIArObjLabelPresentFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag, failed, fic, "SEIArObjConfInfoPresentFlag");
    if (seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag)
    {
      readTokenValueAndValidate<uint32_t>(seiAnnoRegion->m_hdr.m_objectConfidenceLength, failed, fic, "SEIArObjDetConfLength", uint32_t(0), uint32_t(255));
    }
    if (seiAnnoRegion->m_hdr.m_objectLabelPresentFlag)
    {
      readTokenValue(seiAnnoRegion->m_hdr.m_objectLabelLanguagePresentFlag, failed, fic, "SEIArObjLabelLangPresentFlag");
      if (seiAnnoRegion->m_hdr.m_objectLabelLanguagePresentFlag)
      {
        readTokenValue(seiAnnoRegion->m_hdr.m_annotatedRegionsObjectLabelLang, failed, fic, "SEIArLabelLanguage");
      }
      uint32_t numLabelUpdates=0;
      readTokenValueAndValidate<uint32_t>(numLabelUpdates, failed, fic, "SEIArNumLabelUpdates", uint32_t(0), uint32_t(255));
      seiAnnoRegion->m_annotatedLabels.resize(numLabelUpdates);
      for (auto it=seiAnnoRegion->m_annotatedLabels.begin(); it!=seiAnnoRegion->m_annotatedLabels.end(); it++)
      {
        SEIAnnotatedRegions::AnnotatedRegionLabel &ar=it->second;
        readTokenValueAndValidate(it->first, failed, fic, "SEIArLabelIdc[c]", uint32_t(0), uint32_t(255));
        bool cancelFlag;
        readTokenValue(cancelFlag, failed, fic, "SEIArLabelCancelFlag[c]");
        ar.labelValid=!cancelFlag;
        if (ar.labelValid)
        {
          readTokenValue(ar.label, failed, fic, "SEIArLabel[c]");
        }
      }
    }

    uint32_t numObjectUpdates=0;
    readTokenValueAndValidate<uint32_t>(numObjectUpdates, failed, fic, "SEIArNumObjUpdates", uint32_t(0), uint32_t(255));
    seiAnnoRegion->m_annotatedRegions.resize(numObjectUpdates);
    for (auto it=seiAnnoRegion->m_annotatedRegions.begin(); it!=seiAnnoRegion->m_annotatedRegions.end(); it++)
    {
      SEIAnnotatedRegions::AnnotatedRegionObject &ar = it->second;
      readTokenValueAndValidate(it->first, failed, fic, "SEIArObjIdx[c]", uint32_t(0), uint32_t(255));
      readTokenValue(ar.objectCancelFlag, failed, fic, "SEIArObjCancelFlag[c]");
      ar.objectLabelValid=false;
      ar.boundingBoxValid=false;
      ar.boundingBoxCancelFlag=false;

      if (!ar.objectCancelFlag)
      {
        if (seiAnnoRegion->m_hdr.m_objectLabelPresentFlag)
        {
          readTokenValue(ar.objectLabelValid, failed, fic, "SEIArObjLabelUpdateFlag[c]");
          if (ar.objectLabelValid)
          {
            readTokenValueAndValidate<uint32_t>(ar.objLabelIdx, failed, fic, "SEIArObjectLabelIdc[c]", uint32_t(0), uint32_t(255));
          }
        }
        readTokenValue(ar.boundingBoxValid, failed, fic, "SEIArBoundBoxUpdateFlag[c]");
        if (ar.boundingBoxValid)
        {
          readTokenValue(ar.boundingBoxCancelFlag, failed, fic, "SEIArBoundBoxCancelFlag[c]");
          if (!ar.boundingBoxCancelFlag)
          {
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxTop, failed, fic, "SEIArObjTop[c]", uint32_t(0), uint32_t(0x7fffffff));
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxLeft, failed, fic, "SEIArObjLeft[c]", uint32_t(0), uint32_t(0x7fffffff));
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxWidth, failed, fic, "SEIArObjWidth[c]", uint32_t(0), uint32_t(0x7fffffff));
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxHeight, failed, fic, "SEIArObjHeight[c]", uint32_t(0), uint32_t(0x7fffffff));
            if (seiAnnoRegion->m_hdr.m_partialObjectFlagPresentFlag)
            {
              readTokenValue(ar.partialObjectFlag, failed, fic, "SEIArObjPartUpdateFlag[c]");
            }
            if (seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag)
            {
              readTokenValueAndValidate<uint32_t>(ar.objectConfidence, failed, fic, "SEIArObjDetConf[c]", uint32_t(0), uint32_t(1<<seiAnnoRegion->m_hdr.m_objectConfidenceLength)-1);
            }
          }
        }
        //Compare with existing attributes to decide whether it's a static object
        //First check whether it's an existing object (or) new object
        auto destIt = m_pcCfg->m_arObjects.find(it->first);
        //New object
        if (destIt == m_pcCfg->m_arObjects.end())
        {
           //New object arrived, needs to be appended to the map of tracked objects
           m_pcCfg->m_arObjects[it->first] = ar;
        }
        //Existing object
        else
        {
          // Size remains the same
          if(m_pcCfg->m_arObjects[it->first].boundingBoxWidth == ar.boundingBoxWidth &&
            m_pcCfg->m_arObjects[it->first].boundingBoxHeight == ar.boundingBoxHeight)
            {
              if(m_pcCfg->m_arObjects[it->first].boundingBoxTop == ar.boundingBoxTop &&
                m_pcCfg->m_arObjects[it->first].boundingBoxLeft == ar.boundingBoxLeft)
                {
                  ar.boundingBoxValid = 0;
                }
            }
        }
      }
    }
  }
}

bool SEIEncoder::initSEIAnnotatedRegions(SEIAnnotatedRegions* SEIAnnoReg, int currPOC)
{
  assert(m_isInitialized);
  assert(SEIAnnoReg != nullptr);

  // reading external Annotated Regions Information SEI message parameters from file
  if (!m_pcCfg->getAnnotatedRegionSEIFileRoot().empty())
  {
    bool failed = false;
    // building the annotated regions file name with poc num in prefix "_poc.txt"
    std::string AnnoRegionSEIFileWithPoc(m_pcCfg->getAnnotatedRegionSEIFileRoot());
    {
      std::stringstream suffix;
      suffix << "_" << currPOC << ".txt";
      AnnoRegionSEIFileWithPoc += suffix.str();
    }
    std::ifstream fic(AnnoRegionSEIFileWithPoc.c_str());
    if (!fic.good() || !fic.is_open())
    {
      std::cerr << "No Annotated Regions SEI parameters file " << AnnoRegionSEIFileWithPoc << " for POC " << currPOC << std::endl;
      return false;
    }
    //Read annotated region SEI parameters from the cfg file
    readAnnotatedRegionSEI(fic, SEIAnnoReg, failed);
    if (failed)
    {
      std::cerr << "Error while reading Annotated Regions SEI parameters file '" << AnnoRegionSEIFileWithPoc << "'" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  return true;
}


void SEIEncoder::initSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics *seiAltTransCharacteristics)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiAltTransCharacteristics != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAltTransCharacteristics->m_preferredTransferCharacteristics = m_pcCfg->getSEIPreferredTransferCharacteristics();
}
void SEIEncoder::initSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics *seiFilmGrain)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiFilmGrain != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiFilmGrain->m_filmGrainCharacteristicsCancelFlag      = m_pcCfg->getFilmGrainCharactersticsSEICancelFlag();
  seiFilmGrain->m_filmGrainCharacteristicsPersistenceFlag = m_pcCfg->getFilmGrainCharactersticsSEIPersistenceFlag();
  seiFilmGrain->m_filmGrainModelId                        = m_pcCfg->getFilmGrainCharactersticsSEIModelID();
  seiFilmGrain->m_separateColourDescriptionPresentFlag    = m_pcCfg->getFilmGrainCharactersticsSEISepColourDescPresent();
  seiFilmGrain->m_blendingModeId                          = m_pcCfg->getFilmGrainCharactersticsSEIBlendingModeID();
  seiFilmGrain->m_log2ScaleFactor                         = m_pcCfg->getFilmGrainCharactersticsSEILog2ScaleFactor();
  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    seiFilmGrain->m_compModel[i].presentFlag = m_pcCfg->getFGCSEICompModelPresent(i);
    if (seiFilmGrain->m_compModel[i].presentFlag)
    {
      seiFilmGrain->m_compModel[i].numModelValues = 1 + m_pcCfg->getFGCSEINumModelValuesMinus1(i);
      seiFilmGrain->m_compModel[i].numIntensityIntervals = 1 + m_pcCfg->getFGCSEINumIntensityIntervalMinus1(i);
      seiFilmGrain->m_compModel[i].intensityValues.resize(seiFilmGrain->m_compModel[i].numIntensityIntervals);
      for (int j = 0; j < seiFilmGrain->m_compModel[i].numIntensityIntervals; j++)
      {
        seiFilmGrain->m_compModel[i].intensityValues[j].intensityIntervalLowerBound = m_pcCfg->getFGCSEIIntensityIntervalLowerBound(i, j);
        seiFilmGrain->m_compModel[i].intensityValues[j].intensityIntervalUpperBound = m_pcCfg->getFGCSEIIntensityIntervalUpperBound(i, j);
        seiFilmGrain->m_compModel[i].intensityValues[j].compModelValue.resize(seiFilmGrain->m_compModel[i].numModelValues);
        for (int k = 0; k < seiFilmGrain->m_compModel[i].numModelValues; k++)
        {
          seiFilmGrain->m_compModel[i].intensityValues[j].compModelValue[k] = m_pcCfg->getFGCSEICompModelValue(i, j, k);
        }
      }
    }
  }
}

void SEIEncoder::initSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume *seiMDCV)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiMDCV != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  for (int j = 0; j <= 1; j++)
  {
    for (int i = 0; i <= 2; i++)
    {
       seiMDCV->values.primaries[i][j] = m_pcCfg->getMasteringDisplaySEI().primaries[i][j];
    }
    seiMDCV->values.whitePoint[j] = m_pcCfg->getMasteringDisplaySEI().whitePoint[j];
  }
  seiMDCV->values.maxLuminance = m_pcCfg->getMasteringDisplaySEI().maxLuminance;
  seiMDCV->values.minLuminance = m_pcCfg->getMasteringDisplaySEI().minLuminance;
}

void SEIEncoder::initSEIContentLightLevel(SEIContentLightLevelInfo *seiCLL)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiCLL != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiCLL->m_maxContentLightLevel    = m_pcCfg->getCLLSEIMaxContentLightLevel();
  seiCLL->m_maxPicAverageLightLevel = m_pcCfg->getCLLSEIMaxPicAvgLightLevel();
}

void SEIEncoder::initSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment *seiAmbViewEnvironment)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiAmbViewEnvironment != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAmbViewEnvironment->m_ambientIlluminance = m_pcCfg->getAmbientViewingEnvironmentSEIIlluminance();
  seiAmbViewEnvironment->m_ambientLightX      = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightX();
  seiAmbViewEnvironment->m_ambientLightY      = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightY();
}

void SEIEncoder::initSEIContentColourVolume(SEIContentColourVolume *seiContentColourVolume)
{
  assert(m_isInitialized);
  assert(seiContentColourVolume != nullptr);
  seiContentColourVolume->m_ccvCancelFlag = m_pcCfg->getCcvSEICancelFlag();
  seiContentColourVolume->m_ccvPersistenceFlag = m_pcCfg->getCcvSEIPersistenceFlag();

  seiContentColourVolume->m_ccvPrimariesPresentFlag = m_pcCfg->getCcvSEIPrimariesPresentFlag();
  seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMinLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMaxLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag = m_pcCfg->getCcvSEIAvgLuminanceValuePresentFlag();

  // Currently we are using a floor operation for setting up the "integer" values for this SEI.
  // This applies to both primaries and luminance limits.
  if (seiContentColourVolume->m_ccvPrimariesPresentFlag == true)
  {
    for (int i = 0; i < MAX_NUM_COMPONENT; i++)
    {
      seiContentColourVolume->m_ccvPrimariesX[i] = (int32_t)(50000.0 * m_pcCfg->getCcvSEIPrimariesX(i));
      seiContentColourVolume->m_ccvPrimariesY[i] = (int32_t)(50000.0 * m_pcCfg->getCcvSEIPrimariesY(i));
    }
  }

  if (seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMinLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIMinLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMaxLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIMaxLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvAvgLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIAvgLuminanceValue());
  }
}

void SEIEncoder::initSEIScalabilityDimensionInfo(SEIScalabilityDimensionInfo *sei)
{
  CHECK(!(m_isInitialized), "Scalability dimension information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiScalabilityDimensionInfo for initialization (got nullptr)");
  sei->m_sdiMaxLayersMinus1 = m_pcCfg->getSdiSEIMaxLayersMinus1();
  sei->m_sdiMultiviewInfoFlag = m_pcCfg->getSdiSEIMultiviewInfoFlag();
  sei->m_sdiAuxiliaryInfoFlag = m_pcCfg->getSdiSEIAuxiliaryInfoFlag();
  if (sei->m_sdiMultiviewInfoFlag || sei->m_sdiAuxiliaryInfoFlag)
  {
    if (sei->m_sdiMultiviewInfoFlag)
    {
      sei->m_sdiViewIdLenMinus1 = m_pcCfg->getSdiSEIViewIdLenMinus1();
    }
    sei->m_sdiLayerId.resize(sei->m_sdiMaxLayersMinus1 + 1);
    for (int i = 0; i <= sei->m_sdiMaxLayersMinus1; i++)
    {
      sei->m_sdiLayerId[i] = m_pcCfg->getSdiSEILayerId(i);
      sei->m_sdiViewIdVal.resize(sei->m_sdiMaxLayersMinus1 + 1);
      if (sei->m_sdiMultiviewInfoFlag)
      {
        sei->m_sdiViewIdVal[i] = m_pcCfg->getSdiSEIViewIdVal(i);
      }
      sei->m_sdiAuxId.resize(sei->m_sdiMaxLayersMinus1 + 1);
      if (sei->m_sdiAuxiliaryInfoFlag)
      {
        sei->m_sdiAuxId[i] = m_pcCfg->getSdiSEIAuxId(i);
        sei->m_sdiNumAssociatedPrimaryLayersMinus1.resize(sei->m_sdiMaxLayersMinus1 + 1);
        sei->m_sdiAssociatedPrimaryLayerIdx.resize(sei->m_sdiMaxLayersMinus1 + 1);
        if (sei->m_sdiAuxId[i] > 0)
        {
          sei->m_sdiNumAssociatedPrimaryLayersMinus1[i] = m_pcCfg->getSdiSEINumAssociatedPrimaryLayersMinus1(i);
          sei->m_sdiAssociatedPrimaryLayerIdx[i].resize(sei->m_sdiNumAssociatedPrimaryLayersMinus1[i] + 1);
          for (int j = 0; j <= sei->m_sdiNumAssociatedPrimaryLayersMinus1[i]; j++)
          {
            sei->m_sdiAssociatedPrimaryLayerIdx[i][j] = 0;
          }
        }
      }
    }
    sei->m_sdiNumViews = 1;
    if (sei->m_sdiMultiviewInfoFlag)
    {
      for (int i = 1; i <= sei->m_sdiMaxLayersMinus1; i++)
      {
        bool newViewFlag = true;
        for (int j = 0; j < i; j++)
        {
          if (sei->m_sdiViewIdVal[i] == sei->m_sdiViewIdVal[j])
          {
            newViewFlag = false;
          }
        }
        if (newViewFlag)
        {
          sei->m_sdiNumViews++;
        }
      }
    }
  }
}

void SEIEncoder::initSEIMultiviewAcquisitionInfo(SEIMultiviewAcquisitionInfo *sei)
{
  CHECK(!(m_isInitialized), "Multiview acquisition information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiMultiviewAcquisitionInfo for initialization (got nullptr)");
  sei->m_maiIntrinsicParamFlag        = m_pcCfg->getMaiSEIIntrinsicParamFlag();
  sei->m_maiExtrinsicParamFlag        = m_pcCfg->getMaiSEIExtrinsicParamFlag();
  sei->m_maiNumViewsMinus1            = m_pcCfg->getMaiSEINumViewsMinus1();
  if (sei->m_maiIntrinsicParamFlag)
  {
    sei->m_maiIntrinsicParamsEqualFlag  = m_pcCfg->getMaiSEIIntrinsicParamsEqualFlag();
    sei->m_maiPrecFocalLength           = m_pcCfg->getMaiSEIPrecFocalLength();
    sei->m_maiPrecPrincipalPoint        = m_pcCfg->getMaiSEIPrecPrincipalPoint();
    sei->m_maiPrecSkewFactor            = m_pcCfg->getMaiSEIPrecSkewFactor();
    int numViews = sei->m_maiIntrinsicParamsEqualFlag ? 1 : sei->m_maiNumViewsMinus1 + 1;
    sei->m_maiSignFocalLengthX       .resize( numViews );
    sei->m_maiExponentFocalLengthX   .resize( numViews );
    sei->m_maiMantissaFocalLengthX   .resize( numViews );
    sei->m_maiSignFocalLengthY       .resize( numViews );
    sei->m_maiExponentFocalLengthY   .resize( numViews );
    sei->m_maiMantissaFocalLengthY   .resize( numViews );
    sei->m_maiSignPrincipalPointX    .resize( numViews );
    sei->m_maiExponentPrincipalPointX.resize( numViews );
    sei->m_maiMantissaPrincipalPointX.resize( numViews );
    sei->m_maiSignPrincipalPointY    .resize( numViews );
    sei->m_maiExponentPrincipalPointY.resize( numViews );
    sei->m_maiMantissaPrincipalPointY.resize( numViews );
    sei->m_maiSignSkewFactor         .resize( numViews );
    sei->m_maiExponentSkewFactor     .resize( numViews );
    sei->m_maiMantissaSkewFactor     .resize( numViews );
    for( int i = 0; i  <=  ( sei->m_maiIntrinsicParamsEqualFlag ? 0 : sei->m_maiNumViewsMinus1 ); i++ )
    {
      sei->m_maiSignFocalLengthX       [i] = m_pcCfg->getMaiSEISignFocalLengthX(i);
      sei->m_maiExponentFocalLengthX   [i] = m_pcCfg->getMaiSEIExponentFocalLengthX(i);
      sei->m_maiMantissaFocalLengthX   [i] = m_pcCfg->getMaiSEIMantissaFocalLengthX(i);
      sei->m_maiSignFocalLengthY       [i] = m_pcCfg->getMaiSEISignFocalLengthY(i);
      sei->m_maiExponentFocalLengthY   [i] = m_pcCfg->getMaiSEIExponentFocalLengthY(i);
      sei->m_maiMantissaFocalLengthY   [i] = m_pcCfg->getMaiSEIMantissaFocalLengthY(i);
      sei->m_maiSignPrincipalPointX    [i] = m_pcCfg->getMaiSEISignPrincipalPointX(i);
      sei->m_maiExponentPrincipalPointX[i] = m_pcCfg->getMaiSEIExponentPrincipalPointX(i);
      sei->m_maiMantissaPrincipalPointX[i] = m_pcCfg->getMaiSEIMantissaPrincipalPointX(i);
      sei->m_maiSignPrincipalPointY    [i] = m_pcCfg->getMaiSEISignPrincipalPointY(i);
      sei->m_maiExponentPrincipalPointY[i] = m_pcCfg->getMaiSEIExponentPrincipalPointY(i);
      sei->m_maiMantissaPrincipalPointY[i] = m_pcCfg->getMaiSEIMantissaPrincipalPointY(i);
      sei->m_maiSignSkewFactor         [i] = m_pcCfg->getMaiSEISignSkewFactor(i);
      sei->m_maiExponentSkewFactor     [i] = m_pcCfg->getMaiSEIExponentSkewFactor(i);
      sei->m_maiMantissaSkewFactor     [i] = m_pcCfg->getMaiSEIMantissaSkewFactor(i);
    }
  }
  if (sei->m_maiExtrinsicParamFlag)
  {
    sei->m_maiPrecRotationParam = m_pcCfg->getMaiSEIPrecRotationParam();
    sei->m_maiPrecTranslationParam = m_pcCfg->getMaiSEIPrecTranslationParam();
    sei->m_maiSignR.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiExponentR.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiMantissaR.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiSignT.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiExponentT.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiMantissaT.resize(sei->m_maiNumViewsMinus1 + 1);
    for (int i = 0; i <= sei->m_maiNumViewsMinus1; i++)
    {
      sei->m_maiSignR[i].resize(3);
      sei->m_maiExponentR[i].resize(3);
      sei->m_maiMantissaR[i].resize(3);
      sei->m_maiSignT[i].resize(3);
      sei->m_maiExponentT[i].resize(3);
      sei->m_maiMantissaT[i].resize(3);
      for (int j = 0; j < 3; j++)
      {
        sei->m_maiSignR[i][j].resize(3);
        sei->m_maiExponentR[i][j].resize(3);
        sei->m_maiMantissaR[i][j].resize(3);
        for (int k = 0; k < 3; k++)
        {
          sei->m_maiSignR[i][j][k] = 0;
          sei->m_maiExponentR[i][j][k] = 0;
          sei->m_maiMantissaR[i][j][k] = 0;
        }
        sei->m_maiSignT[i][j] = 0;
        sei->m_maiExponentT[i][j] = 0;
        sei->m_maiMantissaT[i][j] = 0;
      }
    }
  }
}

void SEIEncoder::initSEIMultiviewViewPosition(SEIMultiviewViewPosition *sei)
{
  CHECK(!(m_isInitialized), "Multiview view position SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiMultiviewViewPosition for initialization (got nullptr)");
  sei->m_mvpNumViewsMinus1 = m_pcCfg->getMvpSEINumViewsMinus1();

  int numViews = sei->m_mvpNumViewsMinus1 + 1;
  sei->m_mvpViewPosition.resize(numViews);
  for (int i = 0; i <= sei->m_mvpNumViewsMinus1; i++)
  {
    sei->m_mvpViewPosition[i] = m_pcCfg->getMvpSEIViewPosition(i);
  }
}

void SEIEncoder::initSEIAlphaChannelInfo(SEIAlphaChannelInfo *sei)
{
  CHECK(!(m_isInitialized), "Alpha channel information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiAlphaChannelInfo for initialization (got nullptr)");
  sei->m_aciCancelFlag = m_pcCfg->getAciSEICancelFlag();
  sei->m_aciUseIdc = m_pcCfg->getAciSEIUseIdc();
  sei->m_aciBitDepthMinus8 = m_pcCfg->getAciSEIBitDepthMinus8();
  sei->m_aciTransparentValue = m_pcCfg->getAciSEITransparentValue();
  sei->m_aciOpaqueValue = m_pcCfg->getAciSEIOpaqueValue();
  sei->m_aciIncrFlag = m_pcCfg->getAciSEIIncrFlag();
  sei->m_aciClipFlag = m_pcCfg->getAciSEIClipFlag();
  sei->m_aciClipTypeFlag = m_pcCfg->getAciSEIClipTypeFlag();
}

void SEIEncoder::initSEIDepthRepresentationInfo(SEIDepthRepresentationInfo *sei)
{
  CHECK(!(m_isInitialized), "Depth representation information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiDepthRepresentationInfo for initialization (got nullptr)");
  sei->m_driZNearFlag = m_pcCfg->getDriSEIZNearFlag();
  sei->m_driZFarFlag = m_pcCfg->getDriSEIZFarFlag();
  sei->m_driDMinFlag = m_pcCfg->getDriSEIDMinFlag();
  sei->m_driDMaxFlag = m_pcCfg->getDriSEIDMaxFlag();
  sei->m_driZNear = m_pcCfg->getDriSEIZNear();
  sei->m_driZFar = m_pcCfg->getDriSEIZFar();
  sei->m_driDMin = m_pcCfg->getDriSEIDMin();
  sei->m_driDMax = m_pcCfg->getDriSEIDMax();
  sei->m_driDisparityRefViewId = m_pcCfg->getDriSEIDisparityRefViewId();
  sei->m_driDepthRepresentationType = m_pcCfg->getDriSEIDepthRepresentationType();
  sei->m_driDepthNonlinearRepresentationNumMinus1 = m_pcCfg->getDriSEINonlinearNumMinus1();
  sei->m_driDepthNonlinearRepresentationModel.resize(sei->m_driDepthNonlinearRepresentationNumMinus1 + 1);
  for(int i = 0; i < (sei->m_driDepthNonlinearRepresentationNumMinus1 + 1); i++)
  {
    sei->m_driDepthNonlinearRepresentationModel[i] = m_pcCfg->getDriSEINonlinearModel(i);
  }
}

void SEIEncoder::initSEIColourTransformInfo(SEIColourTransformInfo* seiCTI)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiCTI != nullptr), "Unspecified error");

  //  Set SEI message parameters read from command line options
  seiCTI->m_id = m_pcCfg->getCtiSEIId();
  seiCTI->m_signalInfoFlag = m_pcCfg->getCtiSEISignalInfoFlag();
  seiCTI->m_fullRangeFlag = m_pcCfg->getCtiSEIFullRangeFlag();
  seiCTI->m_primaries = m_pcCfg->getCtiSEIPrimaries();
  seiCTI->m_transferFunction = m_pcCfg->getCtiSEITransferFunction();
  seiCTI->m_matrixCoefs = m_pcCfg->getCtiSEIMatrixCoefs();
  seiCTI->m_crossComponentFlag = m_pcCfg->getCtiSEICrossComponentFlag();
  seiCTI->m_crossComponentInferred = m_pcCfg->getCtiSEICrossComponentInferred();
  seiCTI->m_numberChromaLutMinus1 = m_pcCfg->getCtiSEINbChromaLut() - 1;
  seiCTI->m_chromaOffset = m_pcCfg->getCtiSEIChromaOffset();

  seiCTI->m_bitdepth = m_pcCfg->getBitDepth(ChannelType::LUMA);

  for (int i = 0; i < MAX_NUM_COMPONENT; i++) {
    seiCTI->m_lut[i] = m_pcCfg->getCtiSEILut(i);
  }
  seiCTI->m_log2NumberOfPointsPerLut = floorLog2(seiCTI->m_lut[0].numLutValues - 1);
}

void SEIEncoder::initSEISubpictureLevelInfo(SEISubpicureLevelInfo *sei, const SPS *sps)
{
  const EncCfgParam::CfgSEISubpictureLevel &cfgSubPicLevel = m_pcCfg->getSubpicureLevelInfoSEICfg();

  sei->m_sliSublayerInfoPresentFlag = cfgSubPicLevel.m_sliSublayerInfoPresentFlag;
  sei->m_sliMaxSublayers = cfgSubPicLevel.m_sliMaxSublayers;
  sei->m_numRefLevels = cfgSubPicLevel.m_sliSublayerInfoPresentFlag ? (int)cfgSubPicLevel.m_refLevels.size() / cfgSubPicLevel.m_sliMaxSublayers : (int)cfgSubPicLevel.m_refLevels.size();
  sei->m_numSubpics = cfgSubPicLevel.m_numSubpictures;
  sei->m_explicitFractionPresentFlag = cfgSubPicLevel.m_explicitFraction;

  // sei parameters initialization
  sei->m_nonSubpicLayersFraction.resize(sei->m_numRefLevels);
  sei->m_refLevelIdc.resize(sei->m_numRefLevels);
  for (int level = 0; level < sei->m_numRefLevels; level++)
  {
    sei->m_nonSubpicLayersFraction[level].resize(sei->m_sliMaxSublayers);
    sei->m_refLevelIdc[level].resize(sei->m_sliMaxSublayers);
    for (int sublayer = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
    {
      sei->m_refLevelIdc[level][sublayer] = Level::LEVEL15_5;
    }
  }
  if (sei->m_explicitFractionPresentFlag)
  {
    sei->m_refLevelFraction.resize(sei->m_numRefLevels);
    for (int level = 0; level < sei->m_numRefLevels; level++)
    {
      sei->m_refLevelFraction[level].resize(sei->m_numSubpics);
      for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
      {
        sei->m_refLevelFraction[level][subpic].resize(sei->m_sliMaxSublayers);
        for (int sublayer = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
        {
          sei->m_refLevelFraction[level][subpic][sublayer] = 0;
        }
      }
    }
  }

  // set sei parameters according to the configured values
  for (int sublayer = sei->m_sliSublayerInfoPresentFlag ? 0 : sei->m_sliMaxSublayers - 1, cnta = 0, cntb = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
  {
    for (int level = 0; level < sei->m_numRefLevels; level++)
    {
      sei->m_nonSubpicLayersFraction[level][sublayer] = cfgSubPicLevel.m_nonSubpicLayersFraction[cnta];
      sei->m_refLevelIdc[level][sublayer] = cfgSubPicLevel.m_refLevels[cnta++];
      if (sei->m_explicitFractionPresentFlag)
      {
        for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
        {
          sei->m_refLevelFraction[level][subpic][sublayer] = cfgSubPicLevel.m_fractions[cntb++];
        }
      }
    }
  }

  // update the inference of m_refLevelIdc[][] and m_refLevelFraction[][][]
  if (!sei->m_sliSublayerInfoPresentFlag)
  {
    for (int sublayer = sei->m_sliMaxSublayers - 2; sublayer >= 0; sublayer--)
    {
      for (int level = 0; level < sei->m_numRefLevels; level++)
      {
        sei->m_nonSubpicLayersFraction[level][sublayer] = sei->m_nonSubpicLayersFraction[level][sei->m_sliMaxSublayers - 1];
        sei->m_refLevelIdc[level][sublayer] = sei->m_refLevelIdc[level][sei->m_sliMaxSublayers - 1];
        if (sei->m_explicitFractionPresentFlag)
        {
          for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
          {
            sei->m_refLevelFraction[level][subpic][sublayer] = sei->m_refLevelFraction[level][subpic][sei->m_sliMaxSublayers - 1];
          }
        }
      }
    }
  }
}

void SEIEncoder::initSEISEIManifest(SEIManifest *seiSeiManifest, const SEIMessages &seiMessages)
{
  assert(m_isInitialized);
  assert(seiSeiManifest != NULL);
  seiSeiManifest->m_manifestNumSeiMsgTypes = 0;
  for (auto &it: seiMessages)
  {
    seiSeiManifest->m_manifestNumSeiMsgTypes += 1;
    auto tempPayloadType = it->payloadType();
    seiSeiManifest->m_manifestSeiPayloadType.push_back(tempPayloadType);
    auto description = seiSeiManifest->getSEIMessageDescription(tempPayloadType);
    seiSeiManifest->m_manifestSeiDescription.push_back(description);
  }
  CHECK(seiSeiManifest->m_manifestNumSeiMsgTypes == 0, "No SEI messages available");
}

void SEIEncoder::initSEISEIPrefixIndication(SEIPrefixIndication *seiSeiPrefixIndications, const SEI *sei)
{
  assert(m_isInitialized);
  assert(seiSeiPrefixIndications != NULL);
  seiSeiPrefixIndications->m_prefixSeiPayloadType = sei->payloadType(); 
  seiSeiPrefixIndications->m_numSeiPrefixIndicationsMinus1 = seiSeiPrefixIndications->getNumsOfSeiPrefixIndications(sei) - 1; 
  seiSeiPrefixIndications->m_payload = sei;
}

void SEIEncoder::initSEINeuralNetworkPostFilterCharacteristics(SEINeuralNetworkPostFilterCharacteristics *sei, int filterIdx)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(sei != nullptr), "Unspecified error");
  sei->m_purpose = m_pcCfg->getNNPostFilterSEICharacteristicsPurpose(filterIdx);
  sei->m_id = m_pcCfg->getNNPostFilterSEICharacteristicsId(filterIdx);
  sei->m_baseFlag = m_pcCfg->getNNPostFilterSEICharacteristicsBaseFlag(filterIdx);
  sei->m_modeIdc = m_pcCfg->getNNPostFilterSEICharacteristicsModeIdc(filterIdx);
  if (sei->m_modeIdc == POST_FILTER_MODE::URI)
  {
    sei->m_uriTag = m_pcCfg->getNNPostFilterSEICharacteristicsUriTag(filterIdx);
    sei->m_uri    = m_pcCfg->getNNPostFilterSEICharacteristicsUri(filterIdx);
  }
  sei->m_propertyPresentFlag = m_pcCfg->getNNPostFilterSEICharacteristicsPropertyPresentFlag(filterIdx);
  if (sei->m_propertyPresentFlag)
  {
    sei->m_numberInputDecodedPicturesMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsNumberInputDecodedPicturesMinus1(filterIdx);
    CHECK(sei->m_numberInputDecodedPicturesMinus1 > 63, "m_numberInputDecodedPicturesMinus1 shall be in the range of 0 to 63");

    sei->m_inputPicOutputFlag = m_pcCfg->getNNPostFilterSEICharacteristicsInputPicOutputFlag(filterIdx);

    sei->m_absentInputPicZeroFlag = m_pcCfg->getNNPostFilterSEICharacteristicsAbsentInputPicZeroFlag(filterIdx);

    sei->m_numInpPicsInOutputTensor = 0;
    if (sei->m_numberInputDecodedPicturesMinus1 > 0)
    {
      for (uint32_t i = 0; i <= sei->m_numberInputDecodedPicturesMinus1; i++)
      {
        if (sei->m_inputPicOutputFlag[i])
        {
          sei->m_numInpPicsInOutputTensor++;
        }
      }
    }
    else
    {
      sei->m_numInpPicsInOutputTensor = 1;
    }

    if((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      sei->m_outSubCFlag = m_pcCfg->getNNPostFilterSEICharacteristicsOutSubCFlag(filterIdx);
    }
    if((sei->m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      sei->m_outColourFormatIdc = m_pcCfg->getNNPostFilterSEICharacteristicsOutColourFormatIdc(filterIdx);
    }

    const ChromaFormat chromaFormatIdc = m_pcEncLib->getSPS(0)->getChromaFormatIdc();
    uint8_t subWidthC     = SPS::getWinUnitX(chromaFormatIdc);
    uint8_t subHeightC    = SPS::getWinUnitY(chromaFormatIdc);

    uint8_t outSubWidthC  = subWidthC;
    uint8_t outSubHeightC = subHeightC;
    if ((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      if (sei->m_outSubCFlag)
      {
        outSubWidthC  = 1;
        outSubHeightC = 1;
      }
      else
      {
        outSubWidthC  = 2;
        outSubHeightC = 1;
      }
    }
    else if ((sei->m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      CHECK(sei->m_outColourFormatIdc == ChromaFormat::_400, "The value of nnpfc_out_colour_format_idc shall not be equal to 0");
      outSubWidthC  = SPS::getWinUnitX(sei->m_outColourFormatIdc);
      outSubHeightC = SPS::getWinUnitY(sei->m_outColourFormatIdc);
    }

    if((sei->m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      sei->m_picWidthNumeratorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicWidthNumeratorMinus1(filterIdx);
      sei->m_picWidthDenominatorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicWidthDenominatorMinus1(filterIdx);
      sei->m_picHeightNumeratorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicHeightNumeratorMinus1(filterIdx);
      sei->m_picHeightDenominatorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicHeightDenominatorMinus1(filterIdx);
      CHECK(sei->m_picWidthNumeratorMinus1 > 65535, "nnpfc_pic_width_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei->m_picWidthDenominatorMinus1 > 65535, "nnpfc_pic_width_denom_minus1 shall be in the range of 0 to 65535");
      CHECK(sei->m_picHeightNumeratorMinus1 > 65535, "nnpfc_pic_height_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei->m_picHeightDenominatorMinus1 > 65535, "nnpfc_pic_height_denom_minus1 shall be in the range of 0 to 65535");
      int confWinLeftOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowLeftOffset();
      int confWinTopOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowTopOffset();
      int confWinRightOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowRightOffset();
      int confWinBottomOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowBottomOffset();
      int ppsPicWidthInLumaSample  = m_pcEncLib->getPPS(0)->getPicWidthInLumaSamples();
      int ppsPicHeightInLumaSample = m_pcEncLib->getPPS(0)->getPicHeightInLumaSamples();

      
      int croppedWidth = ppsPicWidthInLumaSample - subWidthC * (confWinRightOffset + confWinLeftOffset);
      int croppedHeight = ppsPicHeightInLumaSample - subHeightC * (confWinBottomOffset + confWinTopOffset);
      int outputPicWidth = (int)ceil(((double)croppedWidth * (sei->m_picWidthNumeratorMinus1 + 1)) / (sei->m_picWidthDenominatorMinus1 + 1));
      int outputPicHeight = (int)ceil(((double)croppedHeight * (sei->m_picHeightNumeratorMinus1 + 1)) / (sei->m_picHeightDenominatorMinus1 + 1));

      CHECK(!(outputPicWidth >= croppedWidth && outputPicWidth <= croppedWidth * 16), "output picture width in luma samples shall be in the range of croppedWidth to croppedWidth * 16");
      CHECK(!(outputPicHeight >= croppedHeight && outputPicHeight <= croppedHeight * 16), "output picture height in luma samples shall be in the range of croppedHeight to croppedHeight * 16");

      CHECK((outputPicWidth  % outSubWidthC)  != 0, "The value of nnpfcOutputPicWidth % outSubWidthC shall be equal to 0");
      CHECK((outputPicHeight % outSubHeightC) != 0, "The value of nnpfcOutputPicHeight % outSubHeightC shall be equal to 0");
    }
    if((sei->m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) != 0)
    {
      sei->m_numberInterpolatedPictures = m_pcCfg->getNNPostFilterSEICharacteristicsNumberInterpolatedPictures(filterIdx);
      CHECK(sei->m_numberInputDecodedPicturesMinus1 <= 0, "If nnpfc_purpose is FRAME_RATE_UPSAMPLING, m_numberInputDecodedPicturesMinus1 shall be greater than 0");
    }

    sei->m_componentLastFlag = m_pcCfg->getNNPostFilterSEICharacteristicsComponentLastFlag(filterIdx);
    sei->m_inpFormatIdc = m_pcCfg->getNNPostFilterSEICharacteristicsInpFormatIdc(filterIdx);
    CHECK(sei->m_inpFormatIdc > 255, "The value of nnpfc_inp_format_idc shall be in the range of 0 to 255");
    if (sei->m_inpFormatIdc == 1)
    {
      sei->m_inpTensorBitDepthLumaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsInpTensorBitDepthLumaMinus8(filterIdx);
      sei->m_inpTensorBitDepthChromaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsInpTensorBitDepthChromaMinus8(filterIdx);
    }

    sei->m_inpOrderIdc = m_pcCfg->getNNPostFilterSEICharacteristicsInpOrderIdc(filterIdx);
    CHECK((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0 && sei->m_inpOrderIdc == 0, "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_inp_order_idc shall not be equal to 0");
    sei->m_auxInpIdc             = m_pcCfg->getNNPostFilterSEICharacteristicsAuxInpIdc(filterIdx);


    sei->m_outFormatIdc = m_pcCfg->getNNPostFilterSEICharacteristicsOutFormatIdc(filterIdx);
    CHECK(sei->m_outFormatIdc > 255, "The value of nnpfc_out_format_idc shall be in the range of 0 to 255");
    if (sei->m_outFormatIdc == 1)
    {
      sei->m_outTensorBitDepthLumaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsOutTensorBitDepthLumaMinus8(filterIdx);
      sei->m_outTensorBitDepthChromaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsOutTensorBitDepthChromaMinus8(filterIdx);
    }
    sei->m_sepColDescriptionFlag = m_pcCfg->getNNPostFilterSEICharacteristicsSepColDescriptionFlag(filterIdx);
    if (sei->m_sepColDescriptionFlag)
    {
      sei->m_colPrimaries         = m_pcCfg->getNNPostFilterSEICharacteristicsColPrimaries(filterIdx);
      sei->m_transCharacteristics = m_pcCfg->getNNPostFilterSEICharacteristicsTransCharacteristics(filterIdx);
      if (sei->m_outFormatIdc == 1)
      {
        sei->m_matrixCoeffs         = m_pcCfg->getNNPostFilterSEICharacteristicsMatrixCoeffs(filterIdx);
        CHECK(sei->m_matrixCoeffs == 0 && !(sei->m_outTensorBitDepthChromaMinus8 == sei->m_outTensorBitDepthLumaMinus8 && sei->m_outOrderIdc == 2 && outSubHeightC == 1 && outSubWidthC == 1),
          "nnpfc_matrix_coeffs shall not be equal to 0 unless the following conditions are true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
        CHECK(sei->m_matrixCoeffs == 8 && !((sei->m_outTensorBitDepthChromaMinus8 == sei->m_outTensorBitDepthLumaMinus8) || (sei->m_outTensorBitDepthChromaMinus8 == (sei->m_outTensorBitDepthLumaMinus8 + 1) && sei->m_outOrderIdc == 2 && outSubHeightC == 1 && outSubWidthC == 1)),
          "nnpfc_matrix_coeffs shall not be equal to 8 unless one of the following conditions is true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 or "
          "nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 + 1, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
      }
    }
#if JVET_AD0067_INCLUDE_SYNTAX
    if (sei->m_sepColDescriptionFlag && (sei->m_outFormatIdc == 1))
    {
      sei->m_fullRangeFlag = m_pcCfg->getNNPostFilterSEICharacteristicsFullRangeFlag(filterIdx);
    }
#endif
    sei->m_outOrderIdc = m_pcCfg->getNNPostFilterSEICharacteristicsOutOrderIdc(filterIdx);
    CHECK((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0 && (sei->m_outOrderIdc == 0 || sei->m_outOrderIdc == 3), "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0 or 3");
    CHECK((sei->m_purpose & NNPC_PurposeType::COLOURIZATION) != 0 && sei->m_outOrderIdc == 0, "When nnpfc_purpose & 0x20 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0");
    if(sei->m_outOrderIdc != 0)
    {
      sei->m_chromaLocInfoPresentFlag = m_pcCfg->getNNPostFilterSEICharacteristicsChromaLocInfoPresentFlag(filterIdx);
    }
    else
    {
      sei->m_chromaLocInfoPresentFlag = 0;
    }
      if(sei->m_chromaLocInfoPresentFlag)
      {
        sei->m_chromaSampleLocTypeFrame = m_pcCfg->getNNPostFilterSEICharacteristicsChromaSampleLocTypeFrame(filterIdx);;
      }
    sei->m_constantPatchSizeFlag = m_pcCfg->getNNPostFilterSEICharacteristicsConstantPatchSizeFlag(filterIdx);
    sei->m_patchWidthMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPatchWidthMinus1(filterIdx);
    sei->m_patchHeightMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPatchHeightMinus1(filterIdx);
    if (sei->m_constantPatchSizeFlag == 0)
    {
      sei->m_extendedPatchWidthCdDeltaMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsExtendedPatchWidthCdDeltaMinus1(filterIdx);
      sei->m_extendedPatchHeightCdDeltaMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsExtendedPatchHeightCdDeltaMinus1(filterIdx);
    }
    sei->m_overlap = m_pcCfg->getNNPostFilterSEICharacteristicsOverlap(filterIdx);
    sei->m_paddingType = m_pcCfg->getNNPostFilterSEICharacteristicsPaddingType(filterIdx);
    CHECK((sei->m_paddingType >= 5) && (sei->m_paddingType <= 15), "Reserved nnpfc_padding_type value, shall not be present in bitstreams conforming to this version of VTM");
    CHECK(sei->m_paddingType > 15, "Invalid nnpfc_padding_type value");
    sei->m_lumaPadding = m_pcCfg->getNNPostFilterSEICharacteristicsLumaPadding(filterIdx);
    sei->m_cbPadding = m_pcCfg->getNNPostFilterSEICharacteristicsCbPadding(filterIdx);
    sei->m_crPadding = m_pcCfg->getNNPostFilterSEICharacteristicsCrPadding(filterIdx);

    sei->m_complexityInfoPresentFlag = m_pcCfg->getNNPostFilterSEICharacteristicsComplexityInfoPresentFlag(filterIdx);
    if (sei->m_complexityInfoPresentFlag)
    {
        sei->m_parameterTypeIdc = m_pcCfg->getNNPostFilterSEICharacteristicsParameterTypeIdc(filterIdx);
        sei->m_log2ParameterBitLengthMinus3 = m_pcCfg->getNNPostFilterSEICharacteristicsLog2ParameterBitLengthMinus3(filterIdx);
        sei->m_numParametersIdc = m_pcCfg->getNNPostFilterSEICharacteristicsNumParametersIdc(filterIdx);
        sei->m_numKmacOperationsIdc = m_pcCfg->getNNPostFilterSEICharacteristicsNumKmacOperationsIdc(filterIdx);
        sei->m_totalKilobyteSize = m_pcCfg->getNNPostFilterSEICharacteristicsTotalKilobyteSize(filterIdx);
    }
  }
  if (sei->m_modeIdc == POST_FILTER_MODE::ISO_IEC_15938_17)
  {
    const std::string payloadFilename = m_pcCfg->getNNPostFilterSEICharacteristicsPayloadFilename(filterIdx);
    std::ifstream     bitstreamFile(payloadFilename.c_str(), std::ifstream::in | std::ifstream::binary);
    if (!bitstreamFile)
    {
      EXIT( "Failed to open bitstream file " << payloadFilename.c_str() << " for reading" ) ;
    }

    bitstreamFile.seekg(0, std::ifstream::end);
    sei->m_payloadLength = bitstreamFile.tellg();
    bitstreamFile.seekg(0, std::ifstream::beg);

    sei->m_payloadByte = new char[sei->m_payloadLength];
    bitstreamFile.read(sei->m_payloadByte, sei->m_payloadLength);
    bitstreamFile.close();
  }
}

void SEIEncoder::initSEINeuralNetworkPostFilterActivation(SEINeuralNetworkPostFilterActivation *sei)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(sei != nullptr), "Unspecified error");
  sei->m_targetId = m_pcCfg->getNnPostFilterSEIActivationTargetId();
  sei->m_cancelFlag  = m_pcCfg->getNnPostFilterSEIActivationCancelFlag();
  if(!sei->m_cancelFlag)
  {
    sei->m_persistenceFlag = m_pcCfg->getNnPostFilterSEIActivationPersistenceFlag();
    sei->m_targetBaseFlag = m_pcCfg->getNnPostFilterSEIActivationTargetBaseFlag();
    sei->m_noPrevCLVSFlag = m_pcCfg->getNnPostFilterSEIActivationNoPrevCLVSFlag();
    sei->m_noFollCLVSFlag = m_pcCfg->getNnPostFilterSEIActivationNoFollCLVSFlag();
    sei->m_outputFlag = m_pcCfg->getNnPostFilterSEIActivationOutputFlag();
  }
}


//! \}
