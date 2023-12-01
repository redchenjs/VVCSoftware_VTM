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

#ifndef __SEI__
#define __SEI__

#pragma once
#include <list>
#include <vector>
#include <cstring>

#include "CommonDef.h"
#include "libmd5/MD5.h"

//! \ingroup CommonLib
//! \{
class SPS;

/**
 * Abstract class representing an SEI message with lightweight RTTI.
 */
class SEI
{
public:
  enum class PayloadType : uint16_t
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    FILLER_PAYLOAD                       = 3,
    USER_DATA_REGISTERED_ITU_T_T35       = 4,
    USER_DATA_UNREGISTERED               = 5,
    FILM_GRAIN_CHARACTERISTICS           = 19,
    POST_FILTER_HINT = 22,
    FRAME_PACKING                        = 45,
    DISPLAY_ORIENTATION                  = 47,
    GREEN_METADATA                       = 56,
    PARAMETER_SETS_INCLUSION_INDICATION  = 129,
    DECODING_UNIT_INFO                   = 130,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    COLOUR_TRANSFORM_INFO                = 142,
    CONTENT_LIGHT_LEVEL_INFO             = 144,
    DEPENDENT_RAP_INDICATION             = 145,
    ALTERNATIVE_TRANSFER_CHARACTERISTICS = 147,
    AMBIENT_VIEWING_ENVIRONMENT          = 148,
    CONTENT_COLOUR_VOLUME                = 149,
    EQUIRECTANGULAR_PROJECTION           = 150,
    GENERALIZED_CUBEMAP_PROJECTION       = 153,
    SPHERE_ROTATION                      = 154,
    REGION_WISE_PACKING                  = 155,
    OMNI_VIEWPORT                        = 156,
    ALPHA_CHANNEL_INFO                   = 165,
    FRAME_FIELD_INFO                     = 168,
    DEPTH_REPRESENTATION_INFO            = 177,
    MULTIVIEW_ACQUISITION_INFO           = 179,
    MULTIVIEW_VIEW_POSITION              = 180,
    SEI_MANIFEST = 200,
    SEI_PREFIX_INDICATION = 201,
    SUBPICTURE_LEVEL_INFO                      = 203,
    SAMPLE_ASPECT_RATIO_INFO                   = 204,
    ANNOTATED_REGIONS                          = 202,
    SCALABILITY_DIMENSION_INFO                 = 205,
    EXTENDED_DRAP_INDICATION                   = 206,
    CONSTRAINED_RASL_ENCODING                  = 207,
    VDI_SEI_ENVELOPE                           = 208,
    SHUTTER_INTERVAL_INFO                      = 209,
    NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS = 210,
    NEURAL_NETWORK_POST_FILTER_ACTIVATION      = 211,
    PHASE_INDICATION                           = 212,

    SEI_PROCESSING_ORDER = 213,
#if JVET_AF0310_PO_NESTING
    SEI_PROCESSING_ORDER_NESTING = 214,
#endif
  };

  SEI() {}
  virtual ~SEI() {}

  static const char *getSEIMessageString(SEI::PayloadType payloadType);

  virtual PayloadType payloadType() const = 0;
};

struct SeiPayload
{
  SEI::PayloadType payloadType;
  uint32_t         payloadLayerId;
  bool             payloadNested;
  uint32_t         payloadSize;
  uint8_t         *payload;
  int              duiIdx;
  int              subpicId;
};

typedef std::list<SEI*> SEIMessages;
SEIMessages getSeisByType(const SEIMessages& seiList, SEI::PayloadType seiType);
SEIMessages extractSeisByType(SEIMessages& seiList, SEI::PayloadType seiType);
void deleteSEIs(SEIMessages& seiList);
class SEIFillerPayload : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::FILLER_PAYLOAD; }
  SEIFillerPayload() {}
  SEIFillerPayload(const SEIFillerPayload& sei) {}
  virtual ~SEIFillerPayload() {}

};

class SEIShutterIntervalInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SHUTTER_INTERVAL_INFO; }
  SEIShutterIntervalInfo() {}
  SEIShutterIntervalInfo(const SEIShutterIntervalInfo& sei);
  virtual ~SEIShutterIntervalInfo() {}

  bool                  m_siiEnabled;
  unsigned              m_siiNumUnitsInShutterInterval;
  unsigned              m_siiTimeScale;
  unsigned              m_siiMaxSubLayersMinus1;
  bool                  m_siiFixedSIwithinCLVS;
  std::vector<unsigned> m_siiSubLayerNumUnitsInSI;
};

class SEIProcessingOrderInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SEI_PROCESSING_ORDER; }
  SEIProcessingOrderInfo() {}
  SEIProcessingOrderInfo(const SEIProcessingOrderInfo& sei);
#if JVET_AF0310_PO_NESTING
  virtual ~SEIProcessingOrderInfo() {}
#else
  virtual ~SEIProcessingOrderInfo() { deleteSEIs(m_posWrapSeiMessages); }
#endif

  bool                   m_posEnabled;
#if JVET_AF0061_ADDITION_PO_ID
  uint32_t               m_posId;
#endif
  uint32_t               m_posNumMinus2;
  std::vector<bool>      m_posWrappingFlag;
  std::vector<bool>      m_posImportanceFlag;
  std::vector<bool>      m_posPrefixFlag;
  std::vector<uint16_t>  m_posPayloadType;
  std::vector<uint16_t>   m_posProcessingOrder;
#if JVET_AF0310_PO_NESTING
  std::vector<uint16_t>  m_posNumBitsInPrefix;
#endif
  std::vector<std::vector<uint8_t>> m_posPrefixByte;
#if !JVET_AF0310_PO_NESTING
  SEIMessages            m_posWrapSeiMessages;
#endif
  static bool checkWrappingSEIPayloadType(SEI::PayloadType const payloadType)
  {
    switch (payloadType)
    {
    case SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS:
    case SEI::PayloadType::POST_FILTER_HINT:
    case SEI::PayloadType::CONTENT_LIGHT_LEVEL_INFO:
    case SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS:
    case SEI::PayloadType::COLOUR_TRANSFORM_INFO:
    case SEI::PayloadType::CONTENT_COLOUR_VOLUME:
      return true;
    default:
      return false;
    }
  }
};

#if JVET_AF0310_PO_NESTING
class SEIProcessingOrderNesting : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SEI_PROCESSING_ORDER_NESTING; }
  SEIProcessingOrderNesting() {}
  SEIProcessingOrderNesting(const SEIProcessingOrderNesting& sei);
  virtual ~SEIProcessingOrderNesting() { deleteSEIs(m_ponWrapSeiMessages); }

  std::vector<uint8_t>   m_ponTargetPoId;
  uint32_t               m_ponNumSeisMinus1;
  std::vector<uint8_t>   m_ponProcessingOrder;
  SEIMessages            m_ponWrapSeiMessages;
  std::vector<uint16_t>  m_ponPayloadType;
};
#endif

class SEIEquirectangularProjection : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::EQUIRECTANGULAR_PROJECTION; }

  SEIEquirectangularProjection()  {}
  SEIEquirectangularProjection(const SEIEquirectangularProjection& sei);
  virtual ~SEIEquirectangularProjection() {}

  bool    m_erpCancelFlag;
  bool    m_erpPersistenceFlag;
  bool    m_erpGuardBandFlag;
  uint8_t m_erpGuardBandType;
  uint8_t m_erpLeftGuardBandWidth;
  uint8_t m_erpRightGuardBandWidth;
};

class SEISphereRotation : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SPHERE_ROTATION; }

  SEISphereRotation()  {}
  SEISphereRotation(const SEISphereRotation& sei);
  virtual ~SEISphereRotation() {}

  bool  m_sphereRotationCancelFlag;
  bool  m_sphereRotationPersistenceFlag;
  int   m_sphereRotationYaw;
  int   m_sphereRotationPitch;
  int   m_sphereRotationRoll;
};

class SEIOmniViewport : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::OMNI_VIEWPORT; }

  SEIOmniViewport() {}
  SEIOmniViewport(const SEIOmniViewport& sei);
  virtual ~SEIOmniViewport() {}

  struct OmniViewport
  {
    int      azimuthCentre;
    int      elevationCentre;
    int      tiltCentre;
    uint32_t horRange;
    uint32_t verRange;
  };

  uint32_t m_omniViewportId;
  bool     m_omniViewportCancelFlag;
  bool     m_omniViewportPersistenceFlag;
  uint8_t  m_omniViewportCntMinus1;
  std::vector<OmniViewport> m_omniViewportRegions;
};

class SEIRegionWisePacking : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::REGION_WISE_PACKING; }
  SEIRegionWisePacking() {}
  SEIRegionWisePacking(const SEIRegionWisePacking& sei);
  virtual ~SEIRegionWisePacking() {}
  bool                  m_rwpCancelFlag;
  bool                  m_rwpPersistenceFlag;
  bool                  m_constituentPictureMatchingFlag;
  int                   m_numPackedRegions;
  int                   m_projPictureWidth;
  int                   m_projPictureHeight;
  int                   m_packedPictureWidth;
  int                   m_packedPictureHeight;
  std::vector<uint8_t>  m_rwpTransformType;
  std::vector<bool>     m_rwpGuardBandFlag;
  std::vector<uint32_t> m_projRegionWidth;
  std::vector<uint32_t> m_projRegionHeight;
  std::vector<uint32_t> m_rwpProjRegionTop;
  std::vector<uint32_t> m_projRegionLeft;
  std::vector<uint16_t> m_packedRegionWidth;
  std::vector<uint16_t> m_packedRegionHeight;
  std::vector<uint16_t> m_packedRegionTop;
  std::vector<uint16_t> m_packedRegionLeft;
  std::vector<uint8_t>  m_rwpLeftGuardBandWidth;
  std::vector<uint8_t>  m_rwpRightGuardBandWidth;
  std::vector<uint8_t>  m_rwpTopGuardBandHeight;
  std::vector<uint8_t>  m_rwpBottomGuardBandHeight;
  std::vector<bool>     m_rwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  m_rwpGuardBandType;
};

class SEIGeneralizedCubemapProjection : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::GENERALIZED_CUBEMAP_PROJECTION; }

  SEIGeneralizedCubemapProjection()  {}
  SEIGeneralizedCubemapProjection(const SEIGeneralizedCubemapProjection& sei);
  virtual ~SEIGeneralizedCubemapProjection() {}

  bool                 m_gcmpCancelFlag;
  bool                 m_gcmpPersistenceFlag;
  uint8_t              m_gcmpPackingType;
  uint8_t              m_gcmpMappingFunctionType;
  std::vector<uint8_t> m_gcmpFaceIndex;
  std::vector<uint8_t> m_gcmpFaceRotation;
  std::vector<uint8_t> m_gcmpFunctionCoeffU;
  std::vector<bool>    m_gcmpFunctionUAffectedByVFlag;
  std::vector<uint8_t> m_gcmpFunctionCoeffV;
  std::vector<bool>    m_gcmpFunctionVAffectedByUFlag;
  bool                 m_gcmpGuardBandFlag;
  uint8_t              m_gcmpGuardBandType;
  bool                 m_gcmpGuardBandBoundaryExteriorFlag;
  uint8_t              m_gcmpGuardBandSamplesMinus1;
};

class SEIScalabilityDimensionInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SCALABILITY_DIMENSION_INFO; }
  SEIScalabilityDimensionInfo()
  : m_sdiNumViews (0)
  , m_sdiMaxLayersMinus1 (0)
  , m_sdiMultiviewInfoFlag (false)
  , m_sdiAuxiliaryInfoFlag (false)
  , m_sdiViewIdLenMinus1 (0)
  {
  }
  SEIScalabilityDimensionInfo(const SEIScalabilityDimensionInfo& sei);
  virtual ~SEIScalabilityDimensionInfo() {}
  bool isSDISameContent(SEIScalabilityDimensionInfo* sdiB);

  int                   m_sdiNumViews;
  int                   m_sdiMaxLayersMinus1;
  bool                  m_sdiMultiviewInfoFlag;
  bool                  m_sdiAuxiliaryInfoFlag;
  int                   m_sdiViewIdLenMinus1;
  std::vector<int>      m_sdiLayerId;
  std::vector<int>      m_sdiViewIdVal;
  std::vector<int>      m_sdiAuxId;
  std::vector<int>      m_sdiNumAssociatedPrimaryLayersMinus1;
  std::vector<std::vector<int>> m_sdiAssociatedPrimaryLayerIdx;
};

class SEIMultiviewAcquisitionInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::MULTIVIEW_ACQUISITION_INFO; }
  SEIMultiviewAcquisitionInfo ( ) { };
  SEIMultiviewAcquisitionInfo(const SEIMultiviewAcquisitionInfo& sei);
  ~SEIMultiviewAcquisitionInfo( ) { };
  SEI* getCopy( ) const { return new SEIMultiviewAcquisitionInfo(*this); };
  bool isMAISameContent(SEIMultiviewAcquisitionInfo* maiB);

  void resizeArrays( )
  {
    int numViews = m_maiIntrinsicParamsEqualFlag ? 1 : m_maiNumViewsMinus1 + 1;
    m_maiSignFocalLengthX       .resize( numViews );
    m_maiExponentFocalLengthX   .resize( numViews );
    m_maiMantissaFocalLengthX   .resize( numViews );
    m_maiSignFocalLengthY       .resize( numViews );
    m_maiExponentFocalLengthY   .resize( numViews );
    m_maiMantissaFocalLengthY   .resize( numViews );
    m_maiSignPrincipalPointX    .resize( numViews );
    m_maiExponentPrincipalPointX.resize( numViews );
    m_maiMantissaPrincipalPointX.resize( numViews );
    m_maiSignPrincipalPointY    .resize( numViews );
    m_maiExponentPrincipalPointY.resize( numViews );
    m_maiMantissaPrincipalPointY.resize( numViews );
    m_maiSignSkewFactor         .resize( numViews );
    m_maiExponentSkewFactor     .resize( numViews );
    m_maiMantissaSkewFactor     .resize( numViews );

    m_maiSignR                  .resize( m_maiNumViewsMinus1 + 1 );
    m_maiExponentR              .resize( m_maiNumViewsMinus1 + 1 );
    m_maiMantissaR              .resize( m_maiNumViewsMinus1 + 1 );
    m_maiSignT                  .resize( m_maiNumViewsMinus1 + 1 );
    m_maiExponentT              .resize( m_maiNumViewsMinus1 + 1 );
    m_maiMantissaT              .resize( m_maiNumViewsMinus1 + 1 );

    for( int i = 0; i <= m_maiNumViewsMinus1 ; i++ )
    {
      m_maiSignR    [i].resize( 3 );
      m_maiExponentR[i].resize( 3 );
      m_maiMantissaR[i].resize( 3 );
      m_maiSignT    [i].resize( 3 );
      m_maiExponentT[i].resize( 3 );
      m_maiMantissaT[i].resize( 3 );

      for (int j = 0; j < 3; j++)
      {
        m_maiSignR    [i][j].resize( 3 );
        m_maiExponentR[i][j].resize( 3 );
        m_maiMantissaR[i][j].resize( 3 );
      }
    }
  }

  uint32_t getMantissaFocalLengthXLen   (int i) const;
  uint32_t getMantissaFocalLengthYLen   (int i) const;
  uint32_t getMantissaPrincipalPointXLen(int i) const;
  uint32_t getMantissaPrincipalPointYLen(int i) const;
  uint32_t getMantissaSkewFactorLen     (int i) const;
  uint32_t getMantissaRLen              (int i, int j, int k ) const;
  uint32_t getMantissaTLen              (int i, int j )        const;

  bool              m_maiIntrinsicParamFlag;
  bool              m_maiExtrinsicParamFlag;
  int               m_maiNumViewsMinus1;
  bool              m_maiIntrinsicParamsEqualFlag;
  int               m_maiPrecFocalLength;
  int               m_maiPrecPrincipalPoint;
  int               m_maiPrecSkewFactor;
  std::vector<bool> m_maiSignFocalLengthX;
  std::vector<int>  m_maiExponentFocalLengthX;
  std::vector<int>  m_maiMantissaFocalLengthX;
  std::vector<bool> m_maiSignFocalLengthY;
  std::vector<int>  m_maiExponentFocalLengthY;
  std::vector<int>  m_maiMantissaFocalLengthY;
  std::vector<bool> m_maiSignPrincipalPointX;
  std::vector<int>  m_maiExponentPrincipalPointX;
  std::vector<int>  m_maiMantissaPrincipalPointX;
  std::vector<bool> m_maiSignPrincipalPointY;
  std::vector<int>  m_maiExponentPrincipalPointY;
  std::vector<int>  m_maiMantissaPrincipalPointY;
  std::vector<bool> m_maiSignSkewFactor;
  std::vector<int>  m_maiExponentSkewFactor;
  std::vector<int>  m_maiMantissaSkewFactor;
  int               m_maiPrecRotationParam;
  int               m_maiPrecTranslationParam;
  std::vector< std::vector<std::vector<bool>>> m_maiSignR;
  std::vector< std::vector<std::vector<int>>>  m_maiExponentR;
  std::vector< std::vector<std::vector<int>>>  m_maiMantissaR;
  std::vector< std::vector<bool>> m_maiSignT;
  std::vector< std::vector<int>>  m_maiExponentT;
  std::vector< std::vector<int>>  m_maiMantissaT;
private:
  uint32_t xGetSyntaxElementLen( int expo, int prec, int val ) const;
};

class SEIMultiviewViewPosition : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::MULTIVIEW_VIEW_POSITION; }
  SEIMultiviewViewPosition() { };
  SEIMultiviewViewPosition(const SEIMultiviewViewPosition& sei);
  ~SEIMultiviewViewPosition() { };
  bool isMVPSameContent(SEIMultiviewViewPosition* mvpB);

  int               m_mvpNumViewsMinus1;
  std::vector<int>  m_mvpViewPosition;
};

class SEIAlphaChannelInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::ALPHA_CHANNEL_INFO; }
  SEIAlphaChannelInfo()
  : m_aciCancelFlag (false)
  , m_aciUseIdc (0)
  , m_aciBitDepthMinus8 (0)
  , m_aciTransparentValue (0)
  , m_aciOpaqueValue (255)
  , m_aciIncrFlag (false)
  , m_aciClipFlag (false)
  , m_aciClipTypeFlag (false)
  {};
  SEIAlphaChannelInfo(const SEIAlphaChannelInfo& sei);
  virtual ~SEIAlphaChannelInfo() {};

  bool m_aciCancelFlag;
  int  m_aciUseIdc;
  int  m_aciBitDepthMinus8;
  int  m_aciTransparentValue;
  int  m_aciOpaqueValue;
  bool m_aciIncrFlag;
  bool m_aciClipFlag;
  bool m_aciClipTypeFlag;
};

class SEIDepthRepresentationInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::DEPTH_REPRESENTATION_INFO; }
  SEIDepthRepresentationInfo()
  : m_driZNearFlag (false)
  , m_driZFarFlag (false)
  , m_driDMinFlag (false)
  , m_driDMaxFlag (false)
  , m_driZNear (0.0)
  , m_driZFar (0.0)
  , m_driDMin (0.0)
  , m_driDMax (0.0)
  , m_driDepthRepresentationType (0)
  , m_driDisparityRefViewId (0)
  , m_driDepthNonlinearRepresentationNumMinus1 (0)
  {};
  SEIDepthRepresentationInfo(const SEIDepthRepresentationInfo& sei);
  virtual ~SEIDepthRepresentationInfo() {};

  bool m_driZNearFlag;
  bool m_driZFarFlag;
  bool m_driDMinFlag;
  bool m_driDMaxFlag;
  double m_driZNear;
  double m_driZFar;
  double m_driDMin;
  double m_driDMax;
  int m_driDepthRepresentationType;
  int m_driDisparityRefViewId;
  int m_driDepthNonlinearRepresentationNumMinus1;
  std::vector<int> m_driDepthNonlinearRepresentationModel;
};

class SEISampleAspectRatioInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SAMPLE_ASPECT_RATIO_INFO; }
  SEISampleAspectRatioInfo() {}
  SEISampleAspectRatioInfo(const SEISampleAspectRatioInfo& sei);
  virtual ~SEISampleAspectRatioInfo() {}
  bool                  m_sariCancelFlag;
  bool                  m_sariPersistenceFlag;
  int                   m_sariAspectRatioIdc;
  int                   m_sariSarWidth;
  int                   m_sariSarHeight;
};

class SEIPhaseIndication : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::PHASE_INDICATION; }
  SEIPhaseIndication() {}
  SEIPhaseIndication(const SEIPhaseIndication& sei);
  virtual ~SEIPhaseIndication() {}
  int                   m_horPhaseNum;
  int                   m_horPhaseDenMinus1;
  int                   m_verPhaseNum;
  int                   m_verPhaseDenMinus1;
};

static constexpr uint32_t ISO_IEC_11578_LEN=16;

class SEIuserDataUnregistered : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::USER_DATA_UNREGISTERED; }

  SEIuserDataUnregistered()
    : userData(nullptr)
    {}
    SEIuserDataUnregistered(const SEIuserDataUnregistered& sei);

  virtual ~SEIuserDataUnregistered()
  {
    delete userData;
  }

  uint8_t uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  uint32_t  userDataLength;
  uint8_t *userData;
};

class SEIDecodedPictureHash : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::DECODED_PICTURE_HASH; }

  SEIDecodedPictureHash() {}
  SEIDecodedPictureHash(const SEIDecodedPictureHash& sei);
  virtual ~SEIDecodedPictureHash() {}

  HashType method;
  bool     singleCompFlag;

  PictureHash m_pictureHash;
};

class SEIDependentRAPIndication : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::DEPENDENT_RAP_INDICATION; }
  SEIDependentRAPIndication() { }
  SEIDependentRAPIndication(const SEIDependentRAPIndication& sei) {}

  virtual ~SEIDependentRAPIndication() { }
};


class SEIBufferingPeriod : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::BUFFERING_PERIOD; }
  void copyTo (SEIBufferingPeriod& target) const;

  SEIBufferingPeriod()
  : m_bpNalCpbParamsPresentFlag (false)
  , m_bpVclCpbParamsPresentFlag (false)
  , m_initialCpbRemovalDelayLength (0)
  , m_cpbRemovalDelayLength (0)
  , m_dpbOutputDelayLength (0)
  , m_bpCpbCnt(0)
  , m_duCpbRemovalDelayIncrementLength (0)
  , m_dpbOutputDelayDuLength (0)
  , m_cpbRemovalDelayDeltasPresentFlag (false)
  , m_numCpbRemovalDelayDeltas (0)
  , m_bpMaxSubLayers (0)
  , m_bpDecodingUnitHrdParamsPresentFlag (false)
  , m_decodingUnitCpbParamsInPicTimingSeiFlag (false)
  , m_decodingUnitDpbDuParamsInPicTimingSeiFlag(false)
    , m_sublayerInitialCpbRemovalDelayPresentFlag(false)
    , m_additionalConcatenationInfoPresentFlag (false)
    , m_maxInitialRemovalDelayForConcatenation (0)
    , m_sublayerDpbOutputOffsetsPresentFlag (false)
    , m_altCpbParamsPresentFlag (false)
    , m_useAltCpbParamsFlag (false)
  {
    ::memset(m_initialCpbRemovalDelay, 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalOffset, 0, sizeof(m_initialCpbRemovalOffset));
    ::memset(m_cpbRemovalDelayDelta, 0, sizeof(m_cpbRemovalDelayDelta));
    ::memset(m_dpbOutputTidOffset, 0, sizeof(m_dpbOutputTidOffset));
  }
  SEIBufferingPeriod(const SEIBufferingPeriod& sei);
  virtual ~SEIBufferingPeriod() {}

  void      setDuCpbRemovalDelayIncrementLength( uint32_t value )        { m_duCpbRemovalDelayIncrementLength = value;        }
  uint32_t  getDuCpbRemovalDelayIncrementLength( ) const                 { return m_duCpbRemovalDelayIncrementLength;         }
  void      setDpbOutputDelayDuLength( uint32_t value )                  { m_dpbOutputDelayDuLength = value;                  }
  uint32_t  getDpbOutputDelayDuLength( ) const                           { return m_dpbOutputDelayDuLength;                   }
  bool m_bpNalCpbParamsPresentFlag;
  bool m_bpVclCpbParamsPresentFlag;
  uint32_t m_initialCpbRemovalDelayLength;
  uint32_t m_cpbRemovalDelayLength;
  uint32_t m_dpbOutputDelayLength;
  int      m_bpCpbCnt;
  uint32_t m_duCpbRemovalDelayIncrementLength;
  uint32_t m_dpbOutputDelayDuLength;
  uint32_t m_initialCpbRemovalDelay         [MAX_TLAYER][MAX_CPB_CNT][2];
  uint32_t m_initialCpbRemovalOffset        [MAX_TLAYER][MAX_CPB_CNT][2];
  bool m_concatenationFlag;
  uint32_t m_auCpbRemovalDelayDelta;
  bool m_cpbRemovalDelayDeltasPresentFlag;
  int  m_numCpbRemovalDelayDeltas;
  int  m_bpMaxSubLayers;
  uint32_t m_cpbRemovalDelayDelta    [16];
  bool m_bpDecodingUnitHrdParamsPresentFlag;
  bool m_decodingUnitCpbParamsInPicTimingSeiFlag;
  bool m_decodingUnitDpbDuParamsInPicTimingSeiFlag;
  bool m_sublayerInitialCpbRemovalDelayPresentFlag;
  bool     m_additionalConcatenationInfoPresentFlag;
  uint32_t m_maxInitialRemovalDelayForConcatenation;
  bool     m_sublayerDpbOutputOffsetsPresentFlag;
  uint32_t m_dpbOutputTidOffset      [MAX_TLAYER];
  bool     m_altCpbParamsPresentFlag;
  bool     m_useAltCpbParamsFlag;
};

class SEIPictureTiming : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::PICTURE_TIMING; }
  void copyTo (SEIPictureTiming& target) const;

  SEIPictureTiming()
  : m_picDpbOutputDelay (0)
  , m_picDpbOutputDuDelay (0)
  , m_numDecodingUnitsMinus1 (0)
  , m_duCommonCpbRemovalDelayFlag (false)
  , m_cpbAltTimingInfoPresentFlag (false)
  , m_ptDisplayElementalPeriodsMinus1(0)
  {
    ::memset(m_ptSubLayerDelaysPresentFlag, 0, sizeof(m_ptSubLayerDelaysPresentFlag));
    ::memset(m_duCommonCpbRemovalDelayMinus1, 0, sizeof(m_duCommonCpbRemovalDelayMinus1));
    ::memset(m_cpbRemovalDelayDeltaEnabledFlag, 0, sizeof(m_cpbRemovalDelayDeltaEnabledFlag));
    ::memset(m_cpbRemovalDelayDeltaIdx, 0, sizeof(m_cpbRemovalDelayDeltaIdx));
    ::memset(m_auCpbRemovalDelay, 0, sizeof(m_auCpbRemovalDelay));
  }
  SEIPictureTiming(const SEIPictureTiming& sei);
  virtual ~SEIPictureTiming()
  {
  }


  bool  m_ptSubLayerDelaysPresentFlag[MAX_TLAYER];
  bool  m_cpbRemovalDelayDeltaEnabledFlag[MAX_TLAYER];
  uint32_t  m_cpbRemovalDelayDeltaIdx[MAX_TLAYER];
  uint32_t  m_auCpbRemovalDelay[MAX_TLAYER];
  uint32_t  m_picDpbOutputDelay;
  uint32_t  m_picDpbOutputDuDelay;
  uint32_t  m_numDecodingUnitsMinus1;
  bool  m_duCommonCpbRemovalDelayFlag;
  uint32_t  m_duCommonCpbRemovalDelayMinus1[MAX_TLAYER];
  std::vector<uint32_t> m_numNalusInDuMinus1;
  std::vector<uint32_t> m_duCpbRemovalDelayMinus1;
  bool     m_cpbAltTimingInfoPresentFlag;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_nalCpbDelayOffset;
  std::vector<uint32_t>              m_nalDpbDelayOffset;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_vclCpbDelayOffset;
  std::vector<uint32_t>              m_vclDpbDelayOffset;
  int m_ptDisplayElementalPeriodsMinus1;
};

class SEIDecodingUnitInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::DECODING_UNIT_INFO; }

  SEIDecodingUnitInfo()
    : m_decodingUnitIdx(0)
    , m_dpbOutputDuDelayPresentFlag(false)
    , m_picSptDpbOutputDuDelay(-1)
  {
    ::memset(m_duiSubLayerDelaysPresentFlag, 0, sizeof(m_duiSubLayerDelaysPresentFlag));
    ::memset(m_duSptCpbRemovalDelayIncrement, 0, sizeof(m_duSptCpbRemovalDelayIncrement));
  }
  SEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei);
  virtual ~SEIDecodingUnitInfo() {}
  int m_decodingUnitIdx;
  bool m_duiSubLayerDelaysPresentFlag[MAX_TLAYER];
  int m_duSptCpbRemovalDelayIncrement[MAX_TLAYER];
  bool m_dpbOutputDuDelayPresentFlag;
  int m_picSptDpbOutputDuDelay;
};


class SEIFrameFieldInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::FRAME_FIELD_INFO; }

  SEIFrameFieldInfo()
    : m_fieldPicFlag(false)
    , m_bottomFieldFlag (false)
    , m_pairingIndicatedFlag (false)
    , m_pairedWithNextFieldFlag(false)
    , m_displayFieldsFromFrameFlag(false)
    , m_topFieldFirstFlag(false)
    , m_displayElementalPeriodsMinus1(0)
    , m_sourceScanType(0)
    , m_duplicateFlag(false)
  {}
  SEIFrameFieldInfo(const SEIFrameFieldInfo& sei);
  virtual ~SEIFrameFieldInfo() {}

  bool m_fieldPicFlag;
  bool m_bottomFieldFlag;
  bool m_pairingIndicatedFlag;
  bool m_pairedWithNextFieldFlag;
  bool m_displayFieldsFromFrameFlag;
  bool m_topFieldFirstFlag;
  int  m_displayElementalPeriodsMinus1;
  int  m_sourceScanType;
  bool m_duplicateFlag;
};


class SEIFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::FRAME_PACKING; }

  SEIFramePacking() {}
  SEIFramePacking(const SEIFramePacking& sei);
  virtual ~SEIFramePacking() {}

  int  m_arrangementId;
  bool m_arrangementCancelFlag;
  int  m_arrangementType;
  bool m_quincunxSamplingFlag;
  int  m_contentInterpretationType;
  bool m_spatialFlippingFlag;
  bool m_frame0FlippedFlag;
  bool m_fieldViewsFlag;
  bool m_currentFrameIsFrame0Flag;
  bool m_frame0SelfContainedFlag;
  bool m_frame1SelfContainedFlag;
  int  m_frame0GridPositionX;
  int  m_frame0GridPositionY;
  int  m_frame1GridPositionX;
  int  m_frame1GridPositionY;
  int  m_arrangementReservedByte;
  bool m_arrangementPersistenceFlag;
  bool m_upsampledAspectRatio;
};

class SEIDisplayOrientation : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::DISPLAY_ORIENTATION; }

  SEIDisplayOrientation() {}
  SEIDisplayOrientation(const SEIDisplayOrientation& sei);
  virtual ~SEIDisplayOrientation() {}

  bool                  m_doCancelFlag;
  bool                  m_doPersistenceFlag;
  int                   m_doTransformType;
};

class SEIGreenMetadata : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::GREEN_METADATA; }

  SEIGreenMetadata() {}
  SEIGreenMetadata(const SEIGreenMetadata& sei) {}
  virtual ~SEIGreenMetadata() {}
};


class SEIGreenMetadataInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::GREEN_METADATA; }
  SEIGreenMetadataInfo() {}
  SEIGreenMetadataInfo(const SEIGreenMetadataInfo& sei);

  virtual ~SEIGreenMetadataInfo() {}
  int m_greenMetadataType =-1;
  // Metrics for quality recovery after low-power encoding
  int m_xsdSubpicNumberMinus1 = -1; //xsd_metric_number_minus1 plus 1 indicates the number of objective quality metrics contained in the SEI message.
#if GREEN_METADATA_SEI_ENABLED
  int m_xsdSubPicIdc;
#endif
  
  int     m_xsdMetricValuePSNR;
  int     m_xsdMetricValueSSIM;
  int     m_xsdMetricValueWPSNR;
  int    m_xsdMetricValueWSPSNR;

  bool     m_xsdMetricTypePSNR;
  bool     m_xsdMetricTypeSSIM;
  bool     m_xsdMetricTypeWPSNR;
  bool     m_xsdMetricTypeWSPSNR;

  int      m_greenMetadataGranularityType = 0;
  int      m_greenMetadataExtendedRepresentation = 0;
  int m_periodType= -1;
  int m_numPictures= -1;
  int m_numSeconds = -1;
  SEIComplexityMetrics m_greenComplexityMetrics;
};


class SEIParameterSetsInclusionIndication : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::PARAMETER_SETS_INCLUSION_INDICATION; }
  SEIParameterSetsInclusionIndication() {}
  SEIParameterSetsInclusionIndication(const SEIParameterSetsInclusionIndication& sei);
  virtual ~SEIParameterSetsInclusionIndication() {}

  int m_selfContainedClvsFlag;
};

class SEIMasteringDisplayColourVolume : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::MASTERING_DISPLAY_COLOUR_VOLUME; }
  SEIMasteringDisplayColourVolume() {}
  SEIMasteringDisplayColourVolume(const SEIMasteringDisplayColourVolume& sei);
  virtual ~SEIMasteringDisplayColourVolume() {}

  SEIMasteringDisplay values;
};


class SEIScalableNesting : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SCALABLE_NESTING; }

  SEIScalableNesting()
  : m_snOlsFlag (false)
  , m_snSubpicFlag (false)
  , m_snNumOlssMinus1 (0)
  , m_snAllLayersFlag (false)
  , m_snNumLayersMinus1 (0)
  , m_snNumSubpics (1)
  , m_snSubpicIdLen (0)
  , m_snNumSEIs(0)
  {}
  SEIScalableNesting(const SEIScalableNesting& sei);

  virtual ~SEIScalableNesting()
  {
    deleteSEIs(m_nestedSEIs);
  }

  bool      m_snOlsFlag;
  bool      m_snSubpicFlag;
  uint32_t  m_snNumOlssMinus1;
  uint32_t  m_snOlsIdxDeltaMinus1[MAX_NESTING_NUM_LAYER];
  uint32_t  m_snOlsIdx[MAX_NESTING_NUM_LAYER];
  bool      m_snAllLayersFlag;                      //value valid if m_nestingOlsFlag == 0
  uint32_t  m_snNumLayersMinus1;                    //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0
  uint8_t   m_snLayerId[MAX_NESTING_NUM_LAYER];     //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0. This can e.g. be a static array of 64 uint8_t values
  uint32_t  m_snNumSubpics;
  uint8_t   m_snSubpicIdLen;
  std::vector<uint16_t> m_snSubpicId;
  uint32_t  m_snNumSEIs;

  SEIMessages m_nestedSEIs;
};


#if ENABLE_TRACING
void xTraceSEIHeader();
void xTraceSEIMessageType( SEI::PayloadType payloadType );
#endif

class SEIAlternativeTransferCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::ALTERNATIVE_TRANSFER_CHARACTERISTICS; }

  SEIAlternativeTransferCharacteristics() : m_preferredTransferCharacteristics(18)
  { }
  SEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei);

  virtual ~SEIAlternativeTransferCharacteristics() {}

  uint32_t m_preferredTransferCharacteristics;
};
class SEIUserDataRegistered : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::USER_DATA_REGISTERED_ITU_T_T35; }

  SEIUserDataRegistered() {}
  SEIUserDataRegistered(const SEIUserDataRegistered& sei);
  virtual ~SEIUserDataRegistered() {}

  uint16_t m_ituCountryCode;
  std::vector<uint8_t> m_userData;
};

class SEIFilmGrainCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::FILM_GRAIN_CHARACTERISTICS; }

  SEIFilmGrainCharacteristics() {}
  SEIFilmGrainCharacteristics(const SEIFilmGrainCharacteristics& sei);
  virtual ~SEIFilmGrainCharacteristics() {}

  bool        m_filmGrainCharacteristicsCancelFlag;
  uint8_t     m_filmGrainModelId;
  bool        m_separateColourDescriptionPresentFlag;
  uint8_t     m_filmGrainBitDepthLumaMinus8;
  uint8_t     m_filmGrainBitDepthChromaMinus8;
  bool        m_filmGrainFullRangeFlag;
  uint8_t     m_filmGrainColourPrimaries;
  uint8_t     m_filmGrainTransferCharacteristics;
  uint8_t     m_filmGrainMatrixCoeffs;
  uint8_t     m_blendingModeId;
  uint8_t     m_log2ScaleFactor;

  struct CompModelIntensityValues
  {
    uint8_t intensityIntervalLowerBound;
    uint8_t intensityIntervalUpperBound;
    std::vector<int> compModelValue;
  };

  struct CompModel
  {
    bool  presentFlag;
    uint8_t numModelValues;
    uint8_t numIntensityIntervals;
    std::vector<CompModelIntensityValues> intensityValues;
  };

  CompModel m_compModel[MAX_NUM_COMPONENT];
  bool      m_filmGrainCharacteristicsPersistenceFlag;
};

class SEIContentLightLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::CONTENT_LIGHT_LEVEL_INFO; }
  SEIContentLightLevelInfo() { }
  SEIContentLightLevelInfo(const SEIContentLightLevelInfo& sei);

  virtual ~SEIContentLightLevelInfo() { }

  uint32_t m_maxContentLightLevel;
  uint32_t m_maxPicAverageLightLevel;
};

class SEIAmbientViewingEnvironment : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::AMBIENT_VIEWING_ENVIRONMENT; }
  SEIAmbientViewingEnvironment() { }
  SEIAmbientViewingEnvironment(const SEIAmbientViewingEnvironment& sei);

  virtual ~SEIAmbientViewingEnvironment() { }

  uint32_t m_ambientIlluminance;
  uint16_t m_ambientLightX;
  uint16_t m_ambientLightY;
};

class SEIColourTransformInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::COLOUR_TRANSFORM_INFO; }
  SEIColourTransformInfo() { }
  SEIColourTransformInfo(const SEIColourTransformInfo& sei);
  virtual ~SEIColourTransformInfo() { }

  uint16_t m_id;
  bool     m_signalInfoFlag;
  bool     m_fullRangeFlag;
  uint16_t m_primaries;
  uint16_t m_transferFunction;
  uint16_t m_matrixCoefs;
  bool     m_crossComponentFlag;
  bool     m_crossComponentInferred;
  uint16_t m_numberChromaLutMinus1;
  int      m_chromaOffset;
  uint16_t m_bitdepth;
  uint16_t m_log2NumberOfPointsPerLut;
  LutModel m_lut[MAX_NUM_COMPONENT];
};
class SEIContentColourVolume : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::CONTENT_COLOUR_VOLUME; }
  SEIContentColourVolume() {}
  SEIContentColourVolume(const SEIContentColourVolume& sei);
  virtual ~SEIContentColourVolume() {}

  bool      m_ccvCancelFlag;
  bool      m_ccvPersistenceFlag;
  bool      m_ccvPrimariesPresentFlag;
  bool      m_ccvMinLuminanceValuePresentFlag;
  bool      m_ccvMaxLuminanceValuePresentFlag;
  bool      m_ccvAvgLuminanceValuePresentFlag;
  int       m_ccvPrimariesX[MAX_NUM_COMPONENT];
  int       m_ccvPrimariesY[MAX_NUM_COMPONENT];
  uint32_t  m_ccvMinLuminanceValue;
  uint32_t  m_ccvMaxLuminanceValue;
  uint32_t  m_ccvAvgLuminanceValue;
};

#endif


class SEISubpicureLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SUBPICTURE_LEVEL_INFO; }
  SEISubpicureLevelInfo()
  : m_numRefLevels(0)
  , m_explicitFractionPresentFlag (false)
  , m_cbrConstraintFlag (false)
  , m_numSubpics(0)
  , m_sliMaxSublayers(1)
  , m_sliSublayerInfoPresentFlag(false)
  {}
  SEISubpicureLevelInfo(const SEISubpicureLevelInfo& sei);
  virtual ~SEISubpicureLevelInfo() {}

  int       m_numRefLevels;
  bool      m_explicitFractionPresentFlag;
  bool      m_cbrConstraintFlag;
  int       m_numSubpics;
  int       m_sliMaxSublayers;
  bool      m_sliSublayerInfoPresentFlag;
  std::vector<std::vector<int>>              m_nonSubpicLayersFraction;
  std::vector<std::vector<Level::Name>>      m_refLevelIdc;
  std::vector<std::vector<std::vector<int>>> m_refLevelFraction;
};

class SEIManifest : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SEI_MANIFEST; }

  SEIManifest() {}
  SEIManifest(const SEIManifest& sei);
  virtual ~SEIManifest() {}

  enum SEIManifestDescription
  {
    NO_SEI_MESSAGE           = 0,
    NECESSARY_SEI_MESSAGE    = 1,
    UNNECESSARY_SEI_MESSAGE  = 2,
    UNDETERMINED_SEI_MESSAGE = 3,
  };
  uint16_t                    m_manifestNumSeiMsgTypes;
  std::vector<PayloadType>    m_manifestSeiPayloadType;
  std::vector<uint8_t>        m_manifestSeiDescription;

  SEIManifestDescription getSEIMessageDescription(const PayloadType payloadType);
};

class SEIPrefixIndication : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::SEI_PREFIX_INDICATION; }

  SEIPrefixIndication() {}
  SEIPrefixIndication(const SEIPrefixIndication& sei);
  virtual ~SEIPrefixIndication() {}

  PayloadType                   m_prefixSeiPayloadType;
  uint8_t                       m_numSeiPrefixIndicationsMinus1;
  std::vector<uint16_t>         m_numBitsInPrefixIndicationMinus1;
  std::vector<std::vector<int>> m_seiPrefixDataBit;
  const SEI*                    m_payload;

  uint8_t getNumsOfSeiPrefixIndications(const SEI *sei);
};

class SEIAnnotatedRegions : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::ANNOTATED_REGIONS; }
  SEIAnnotatedRegions() {}
  SEIAnnotatedRegions(const SEIAnnotatedRegions& sei)
  {
    copyFrom(sei);
  }
  virtual ~SEIAnnotatedRegions() {}

  void copyFrom(const SEIAnnotatedRegions &seiAnnotatedRegions)
  {
    (*this) = seiAnnotatedRegions;
  }

  struct AnnotatedRegionObject
  {
    AnnotatedRegionObject() :
      objectCancelFlag(false),
      objectLabelValid(false),
      boundingBoxValid(false)
    { }
    bool objectCancelFlag;

    bool objectLabelValid;
    uint32_t objLabelIdx;            // only valid if bObjectLabelValid

    bool boundingBoxValid;
    bool boundingBoxCancelFlag;
    uint32_t boundingBoxTop;         // only valid if bBoundingBoxValid
    uint32_t boundingBoxLeft;
    uint32_t boundingBoxWidth;
    uint32_t boundingBoxHeight;

    bool partialObjectFlag;        // only valid if bPartialObjectFlagValid
    uint32_t objectConfidence;
  };
  struct AnnotatedRegionLabel
  {
    AnnotatedRegionLabel() : labelValid(false) { }
    bool        labelValid;
    std::string label;           // only valid if bLabelValid
  };

  struct AnnotatedRegionHeader
  {
    AnnotatedRegionHeader() : m_cancelFlag(true), m_receivedSettingsOnce(false) { }
    bool      m_cancelFlag;
    bool      m_receivedSettingsOnce; // used for decoder conformance checking. Other confidence flags must be unchanged once this flag is set.

    bool      m_notOptimizedForViewingFlag;
    bool      m_trueMotionFlag;
    bool      m_occludedObjectFlag;
    bool      m_partialObjectFlagPresentFlag;
    bool      m_objectLabelPresentFlag;
    bool      m_objectConfidenceInfoPresentFlag;
    uint32_t  m_objectConfidenceLength;         // Only valid if m_objectConfidenceInfoPresentFlag
    bool      m_objectLabelLanguagePresentFlag; // Only valid if m_objectLabelPresentFlag

    std::string m_annotatedRegionsObjectLabelLang;
  };
  typedef uint32_t AnnotatedRegionObjectIndex;
  typedef uint32_t AnnotatedRegionLabelIndex;

  AnnotatedRegionHeader m_hdr;
  std::vector<std::pair<AnnotatedRegionObjectIndex, AnnotatedRegionObject> > m_annotatedRegions;
  std::vector<std::pair<AnnotatedRegionLabelIndex,  AnnotatedRegionLabel>  > m_annotatedLabels;
};

class SEIExtendedDrapIndication : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::EXTENDED_DRAP_INDICATION; }

  SEIExtendedDrapIndication() {}
  SEIExtendedDrapIndication(const SEIExtendedDrapIndication& sei);
  virtual ~SEIExtendedDrapIndication() {}

  int               m_edrapIndicationRapIdMinus1;
  bool              m_edrapIndicationLeadingPicturesDecodableFlag;
  int               m_edrapIndicationReservedZero12Bits;
  int               m_edrapIndicationNumRefRapPicsMinus1;
  std::vector<int>  m_edrapIndicationRefRapId;
};

class SEIConstrainedRaslIndication : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::CONSTRAINED_RASL_ENCODING; }
  SEIConstrainedRaslIndication() { }
  SEIConstrainedRaslIndication(const SEIConstrainedRaslIndication& sei) {}

  virtual ~SEIConstrainedRaslIndication() { }
};

class SEIVDISeiEnvelope : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::VDI_SEI_ENVELOPE; }

  SEIVDISeiEnvelope() {}
  SEIVDISeiEnvelope(const SEIVDISeiEnvelope& sei) {}
  virtual ~SEIVDISeiEnvelope() {}
};

class SEINeuralNetworkPostFilterCharacteristics : public SEI
{
public:
  PayloadType payloadType() const override { return PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS; }
  SEINeuralNetworkPostFilterCharacteristics()
    : m_id(0)
    , m_modeIdc(0)
    , m_propertyPresentFlag(false)
    , m_purpose(0)
    , m_baseFlag(false)
    , m_outSubCFlag(0)
    , m_outSubWidthC(1)
    , m_outSubHeightC(1)
    , m_outColourFormatIdc(ChromaFormat::_420)
    , m_chromaLocInfoPresentFlag(false)
    , m_chromaSampleLocTypeFrame(Chroma420LocType::UNSPECIFIED)
    , m_picWidthNumeratorMinus1(0)
    , m_picWidthDenominatorMinus1(0)
    , m_picHeightNumeratorMinus1(0)
    , m_picHeightDenominatorMinus1(0)
    , m_picWidthInLumaSamples(0)
    , m_picHeightInLumaSamples(0)
    , m_inpTensorBitDepthLumaMinus8(0)
    , m_inpTensorBitDepthChromaMinus8(0)
    , m_outTensorBitDepthLumaMinus8(0)
    , m_outTensorBitDepthChromaMinus8(0)
    , m_componentLastFlag(false)
    , m_inpFormatIdc(0)
    , m_auxInpIdc(0)
    , m_sepColDescriptionFlag(false)
#if JVET_AD0067_INCLUDE_SYNTAX
    , m_fullRangeFlag(false)
#endif
    , m_colPrimaries(0)
    , m_transCharacteristics(0)
    , m_matrixCoeffs(0)
    , m_inpOrderIdc(0)
    , m_outFormatIdc(0)
    , m_outOrderIdc(0)
    , m_constantPatchSizeFlag(false)
    , m_patchWidthMinus1(0)
    , m_patchHeightMinus1(0)
    , m_extendedPatchWidthCdDeltaMinus1(0)
    , m_extendedPatchHeightCdDeltaMinus1(0)
    , m_overlap(0)
    , m_paddingType(0)
    , m_lumaPadding(0)
    , m_cbPadding(0)
    , m_crPadding(0)
    , m_payloadByte(nullptr)
    , m_complexityInfoPresentFlag(false)
    , m_uriTag("")
    , m_uri("")
    , m_parameterTypeIdc(0)
    , m_log2ParameterBitLengthMinus3(0)
    , m_numParametersIdc(0)
    , m_numKmacOperationsIdc(0)
    , m_totalKilobyteSize(0)
    , m_numberInputDecodedPicturesMinus1(0)
    , m_absentInputPicZeroFlag(false)
    , m_numInpPicsInOutputTensor(0)
  {}
  SEINeuralNetworkPostFilterCharacteristics(const SEINeuralNetworkPostFilterCharacteristics& sei);

  bool operator == (const SEINeuralNetworkPostFilterCharacteristics& sei);

  ~SEINeuralNetworkPostFilterCharacteristics() override
  {
    if (m_payloadByte)
    {
      delete m_payloadByte;
      m_payloadByte = nullptr;
    }
  }

  uint32_t       m_id;
  uint32_t       m_modeIdc;
  bool           m_propertyPresentFlag;
  uint32_t       m_purpose;
  bool           m_baseFlag;
  bool           m_outSubCFlag;
  uint8_t        m_outSubWidthC;
  uint8_t        m_outSubHeightC;
  ChromaFormat   m_outColourFormatIdc;
  bool           m_chromaLocInfoPresentFlag;
  Chroma420LocType m_chromaSampleLocTypeFrame;
  uint32_t       m_picWidthNumeratorMinus1;
  uint32_t       m_picWidthDenominatorMinus1;
  uint32_t       m_picHeightNumeratorMinus1;
  uint32_t       m_picHeightDenominatorMinus1;
  uint32_t       m_picWidthInLumaSamples;
  uint32_t       m_picHeightInLumaSamples;
  uint32_t       m_inpTensorBitDepthLumaMinus8;
  uint32_t       m_inpTensorBitDepthChromaMinus8;
  uint32_t       m_outTensorBitDepthLumaMinus8;
  uint32_t       m_outTensorBitDepthChromaMinus8;
  bool           m_componentLastFlag;
  uint32_t       m_inpFormatIdc;
  uint32_t m_auxInpIdc;
  bool     m_sepColDescriptionFlag;
#if JVET_AD0067_INCLUDE_SYNTAX
  bool           m_fullRangeFlag;
#endif
  uint8_t  m_colPrimaries;
  uint8_t  m_transCharacteristics;
  uint8_t  m_matrixCoeffs;
  uint32_t       m_inpOrderIdc;
  uint32_t       m_outFormatIdc;
  uint32_t       m_outOrderIdc;
  bool           m_constantPatchSizeFlag;
  uint32_t       m_patchWidthMinus1;
  uint32_t       m_patchHeightMinus1;
  uint32_t       m_extendedPatchWidthCdDeltaMinus1;
  uint32_t       m_extendedPatchHeightCdDeltaMinus1;
  uint32_t       m_overlap;
  uint32_t       m_paddingType;
  uint32_t       m_lumaPadding;
  uint32_t       m_cbPadding;
  uint32_t       m_crPadding;
  uint64_t       m_payloadLength;
  char*          m_payloadByte;
  bool           m_complexityInfoPresentFlag;
  std::string    m_uriTag;
  std::string    m_uri;
  uint32_t       m_parameterTypeIdc;
  uint32_t       m_log2ParameterBitLengthMinus3;
  uint32_t       m_numParametersIdc;
  uint32_t       m_numKmacOperationsIdc;
  uint32_t       m_totalKilobyteSize;
  uint32_t       m_numberInputDecodedPicturesMinus1;
  std::vector<uint32_t> m_numberInterpolatedPictures;
  std::vector<bool> m_inputPicOutputFlag;
  bool           m_absentInputPicZeroFlag;
  uint32_t       m_numInpPicsInOutputTensor;
};

class SEINeuralNetworkPostFilterActivation : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::NEURAL_NETWORK_POST_FILTER_ACTIVATION; }
  SEINeuralNetworkPostFilterActivation()
    : m_targetId(0)
    , m_cancelFlag(false)
    , m_targetBaseFlag(false)
    , m_noPrevCLVSFlag(false)
    , m_noFollCLVSFlag(false)
    , m_persistenceFlag(false)
  {}
  SEINeuralNetworkPostFilterActivation(const SEINeuralNetworkPostFilterActivation& sei);

  virtual ~SEINeuralNetworkPostFilterActivation() {}

  uint32_t       m_targetId;
  bool           m_cancelFlag;
  bool           m_targetBaseFlag;
  bool           m_noPrevCLVSFlag;
  bool           m_noFollCLVSFlag;
  bool           m_persistenceFlag;
  std::vector<bool> m_outputFlag;
};

class SEIPostFilterHint : public SEI
{
public:
  PayloadType payloadType() const { return PayloadType::POST_FILTER_HINT; }

  SEIPostFilterHint() {}
  SEIPostFilterHint(const SEIPostFilterHint& sei);
  virtual ~SEIPostFilterHint() {}

  bool             m_filterHintCancelFlag;
  bool             m_filterHintPersistenceFlag;
  uint32_t         m_filterHintSizeY;
  uint32_t         m_filterHintSizeX;
  uint32_t         m_filterHintType;
  bool             m_filterHintChromaCoeffPresentFlag;
  std::vector<int> m_filterHintValues;   // values stored in linear array, [ ( ( component * sizeY + y ) * SizeX ) + x ]
};

SEINeuralNetworkPostFilterCharacteristics* getNnpfcWithGivenId(const SEIMessages &seiList, uint32_t nnpfaTargetId);
SEINeuralNetworkPostFilterCharacteristics* getSuperResolutionNnpfc(const SEIMessages &seiList);
//! \}


