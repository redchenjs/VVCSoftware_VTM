/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"

const int CoeffCodingContext::prefixCtx[8] = { 0, 0, 0, 3, 6, 10, 15, 21 };

CoeffCodingContext::CoeffCodingContext(const TransformUnit &tu, ComponentID component, bool signHide,
                                       const BdpcmMode bdpcm)
  : m_compID(component)
  , m_chType(toChannelType(m_compID))
  , m_width(tu.block(m_compID).width)
  , m_height(tu.block(m_compID).height)
  , m_log2CGWidth(g_log2SbbSize[floorLog2(m_width)][floorLog2(m_height)][0])
  , m_log2CGHeight(g_log2SbbSize[floorLog2(m_width)][floorLog2(m_height)][1])
  , m_log2CGSize(m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups(getNonzeroTuSize(m_width) >> m_log2CGWidth)
  , m_heightInGroups(getNonzeroTuSize(m_height) >> m_log2CGHeight)
  , m_log2BlockWidth((unsigned) floorLog2(m_width))
  , m_log2BlockHeight((unsigned) floorLog2(m_height))
  , m_maxNumCoeff(m_width * m_height)
  , m_signHiding(signHide)
  , m_extendedPrecision(tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  , m_maxLog2TrDynamicRange(tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scan(g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(m_width)]
                      [gp_sizeIdxInfo->idxFrom(m_height)])
  , m_scanCG(g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(m_widthInGroups)]
                        [gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
  , m_CtxSetLastX(Ctx::LastX[to_underlying(m_chType)])
  , m_CtxSetLastY(Ctx::LastY[to_underlying(m_chType)])
  , m_maxLastPosX(g_groupIdx[getNonzeroTuSize(m_width) - 1])
  , m_maxLastPosY(g_groupIdx[getNonzeroTuSize(m_height) - 1])
  , m_lastOffsetX(0)
  , m_lastOffsetY(0)
  , m_lastShiftX(0)
  , m_lastShiftY(0)
  , m_minCoeff(-(1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)))
  , m_maxCoeff((1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)) - 1)
  , m_scanPosLast(-1)
  , m_subSetId(-1)
  , m_subSetPos(-1)
  , m_subSetPosX(-1)
  , m_subSetPosY(-1)
  , m_minSubPos(-1)
  , m_maxSubPos(-1)
  , m_sigGroupCtxId(-1)
  , m_tmplCpSum1(-1)
  , m_tmplCpDiag(-1)
  , m_sigFlagCtxSet{ Ctx::SigFlag[to_underlying(m_chType)], Ctx::SigFlag[to_underlying(m_chType) + 2],
                     Ctx::SigFlag[to_underlying(m_chType) + 4] }
  , m_parFlagCtxSet(Ctx::ParFlag[to_underlying(m_chType)])
  , m_gtxFlagCtxSet{ Ctx::GtxFlag[to_underlying(m_chType)], Ctx::GtxFlag[to_underlying(m_chType) + 2] }
  , m_sigGroupCtxIdTS(-1)
  , m_tsSigFlagCtxSet(Ctx::TsSigFlag)
  , m_tsParFlagCtxSet(Ctx::TsParFlag)
  , m_tsGtxFlagCtxSet(Ctx::TsGtxFlag)
  , m_tsLrg1FlagCtxSet(Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet(Ctx::TsResidualSign)
  , m_sigCoeffGroupFlag()
  , m_bdpcm(bdpcm)
{
  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
  if (m_chType == ChannelType::CHROMA)
  {
    const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( m_width  >> 3) );
    const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( m_height >> 3) );
  }
  else
  {
    const_cast<int &>(m_lastOffsetX) = prefixCtx[log2sizeX];
    const_cast<int &>(m_lastOffsetY) = prefixCtx[log2sizeY];

    const_cast<int&>(m_lastShiftX)  = (log2sizeX + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (log2sizeY + 1) >> 2;
  }

  m_cctxBaseLevel = 4; // default value for RRC rice derivation in VVCv1, is updated for extended RRC rice derivation
  m_histValue = 0;  // default value for RRC rice derivation in VVCv1, is updated for history-based extention of RRC rice derivation
  m_updateHist = 0;  // default value for RRC rice derivation (history update is disabled), is updated for history-based extention of RRC rice derivation

  if (tu.cs->sps->getSpsRangeExtension().getRrcRiceExtensionEnableFlag())
  {
    deriveRiceRRC = &CoeffCodingContext::deriveRiceExt;
  }
  else
  {
    deriveRiceRRC = &CoeffCodingContext::deriveRice;
  }
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[m_subSetId].idx;
  m_subSetPosY              = m_subSetPos / m_widthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[to_underlying(m_chType)](sigRight | sigLower);
  unsigned  sigLeft   = unsigned( CGPosX > 0 ? m_sigCoeffGroupFlag[m_subSetPos - 1              ] : false );
  unsigned  sigAbove  = unsigned( CGPosY > 0 ? m_sigCoeffGroupFlag[m_subSetPos - m_widthInGroups] : false );
  m_sigGroupCtxIdTS   = Ctx::TsSigCoeffGroup( sigLeft  + sigAbove );
}


unsigned DeriveCtx::CtxModeConsFlag( const CodingStructure& cs, Partitioner& partitioner )
{
  assert(isLuma(partitioner.chType));
  const Position pos         = partitioner.currArea().block(partitioner.chType);
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  unsigned ctxId = ((cuAbove && CU::isIntra(*cuAbove)) || (cuLeft && CU::isIntra(*cuLeft))) ? 1 : 0;
  return ctxId;
}


void DeriveCtx::CtxSplit( const CodingStructure& cs, Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, bool* _canSplit /*= nullptr */ )
{
  const Position pos         = partitioner.currArea().block(partitioner.chType);
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx  = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  // get left depth
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  // get above depth
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  bool canSplit[6];

  if( _canSplit == nullptr )
  {
    partitioner.canSplit( cs, canSplit[0], canSplit[1], canSplit[2], canSplit[3], canSplit[4], canSplit[5] );
  }
  else
  {
    memcpy( canSplit, _canSplit, 6 * sizeof( bool ) );
  }

  ///////////////////////
  // CTX do split (0-8)
  ///////////////////////
  const unsigned widthCurr  = partitioner.currArea().block(partitioner.chType).width;
  const unsigned heightCurr = partitioner.currArea().block(partitioner.chType).height;

  ctxSpl = 0;

  if( cuLeft )
  {
    const unsigned heightLeft = cuLeft->block(partitioner.chType).height;
    ctxSpl += ( heightLeft < heightCurr ? 1 : 0 );
  }
  if( cuAbove )
  {
    const unsigned widthAbove = cuAbove->block(partitioner.chType).width;
    ctxSpl += ( widthAbove < widthCurr ? 1 : 0 );
  }

  unsigned numSplit = 0;
  if (canSplit[1])
  {
    numSplit += 2;
  }
  if (canSplit[2])
  {
    numSplit += 1;
  }
  if (canSplit[3])
  {
    numSplit += 1;
  }
  if (canSplit[4])
  {
    numSplit += 1;
  }
  if (canSplit[5])
  {
    numSplit += 1;
  }

  if (numSplit > 0)
  {
    numSplit--;
  }

  ctxSpl += 3 * ( numSplit >> 1 );

  //////////////////////////
  // CTX is qt split (0-5)
  //////////////////////////
  ctxQt =  ( cuLeft  && cuLeft->qtDepth  > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += partitioner.currQtDepth < 2 ? 0 : 3;

  ////////////////////////////
  // CTX is ver split (0-4)
  ////////////////////////////
  ctxHv = 0;

  const unsigned numHor = ( canSplit[2] ? 1 : 0 ) + ( canSplit[4] ? 1 : 0 );
  const unsigned numVer = ( canSplit[3] ? 1 : 0 ) + ( canSplit[5] ? 1 : 0 );

  if( numVer == numHor )
  {
    const Area &area = partitioner.currArea().block(partitioner.chType);

    const unsigned wAbove = cuAbove ? cuAbove->block(partitioner.chType).width : 1;
    const unsigned hLeft  = cuLeft ? cuLeft->block(partitioner.chType).height : 1;

    const unsigned depAbove     = area.width / wAbove;
    const unsigned depLeft      = area.height / hLeft;

    if (depAbove == depLeft || !cuLeft || !cuAbove)
    {
      ctxHv = 0;
    }
    else if (depAbove < depLeft)
    {
      ctxHv = 1;
    }
    else
    {
      ctxHv = 2;
    }
  }
  else if( numVer < numHor )
  {
    ctxHv = 3;
  }
  else
  {
    ctxHv = 4;
  }

  //////////////////////////
  // CTX is h/v bt (0-3)
  //////////////////////////
  ctxHorBt = ( partitioner.currMtDepth <= 1 ? 1 : 0 );
  ctxVerBt = ( partitioner.currMtDepth <= 1 ? 3 : 2 );
}

unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const bool prevCbf, const int ispIdx )
{
  if( ispIdx && isLuma( compID ) )
  {
    return 2 + (int)prevCbf;
  }
  if( compID == COMPONENT_Cr )
  {
    return ( prevCbf ? 1 : 0 );
  }
  return 0;
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
  return ( 7 - ((floorLog2(pu.lumaSize().width) + floorLog2(pu.lumaSize().height) + 1) >> 1) );
}

unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, ChannelType::LUMA);
  ctxId = ( cuLeft && cuLeft->affine ) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, ChannelType::LUMA);
  ctxId += ( cuAbove && cuAbove->affine ) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, ChannelType::LUMA);
  ctxId = ( cuLeft && cuLeft->skip ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, ChannelType::LUMA);
  ctxId += ( cuAbove && cuAbove->skip ) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxPredModeFlag( const CodingUnit& cu )
{
  const CodingUnit *cuLeft  = cu.cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, ChannelType::LUMA);
  const CodingUnit *cuAbove = cu.cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, ChannelType::LUMA);

  unsigned ctxId = ((cuAbove && CU::isIntra(*cuAbove)) || (cuLeft && CU::isIntra(*cuLeft))) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxIBCFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;
  const Position         pos    = cu.chType == ChannelType::CHROMA ? cu.chromaPos() : cu.lumaPos();
  const CodingUnit *cuLeft = cs->getCURestricted(pos.offset(-1, 0), cu, cu.chType);
  ctxId += (cuLeft && CU::isIBC(*cuLeft)) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(pos.offset(0, -1), cu, cu.chType);
  ctxId += (cuAbove && CU::isIBC(*cuAbove)) ? 1 : 0;
  return ctxId;
}

void MergeCtx::setMergeInfo( PredictionUnit& pu, int candIdx )
{
  CHECK( candIdx >= numValidMergeCand, "Merge candidate does not exist" );
  pu.regularMergeFlag        = !(pu.ciipFlag || pu.cu->geoFlag);
  pu.mergeFlag               = true;
  pu.mmvdMergeFlag = false;
  pu.interDir                = interDirNeighbours[candIdx];
  pu.cu->imv = (!pu.cu->geoFlag && useAltHpelIf[candIdx]) ? IMV_HPEL : 0;
  pu.mergeIdx                = candIdx;
  pu.mergeType               = CU::isIBC(*pu.cu) ? MergeType::IBC : MergeType::DEFAULT_N;

  for (const auto l: { REF_PIC_LIST_0, REF_PIC_LIST_1 })
  {
    pu.mv[l]     = mvFieldNeighbours[candIdx][l].mv;
    pu.mvd[l]    = Mv();
    pu.refIdx[l] = mvFieldNeighbours[candIdx][l].refIdx;
    pu.mvpIdx[l] = NOT_VALID;
    pu.mvpNum[l] = NOT_VALID;
  }
#if GDR_ENABLED
  CodingStructure &cs = *pu.cs;
  const bool       isEncodeGdrClean =
    cs.sps->getGDREnabledFlag() && cs.pcv->isEncoder
    && ((cs.picHeader->getInGdrInterval() && cs.isClean(pu.Y().topRight(), ChannelType::LUMA))
        || (cs.picHeader->getNumVerVirtualBoundaries() == 0));

  if (isEncodeGdrClean)
  {
    for (const auto l: { REF_PIC_LIST_0, REF_PIC_LIST_1 })
    {
      Mv mv = pu.mv[l];

      int refIdx = pu.refIdx[l];

      pu.mvSolid[l] = mvSolid[candIdx][l];
      pu.mvValid[l] = cs.isClean(pu.Y().topRight(), mv, l, refIdx);
    }
  }
#endif

  if (CU::isIBC(*pu.cu))
  {
    pu.bv = pu.mv[REF_PIC_LIST_0];
    pu.bv.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);   // used for only integer resolution
    pu.cu->imv = pu.cu->imv == IMV_HPEL ? 0 : pu.cu->imv;
  }
  pu.cu->bcwIdx = (interDirNeighbours[candIdx] == 3) ? bcwIdx[candIdx] : BCW_DEFAULT;

  PU::restrictBiPredMergeCandsOne(pu);
  pu.mmvdEncOptMode = 0;
}

void MergeCtx::getMmvdDeltaMv(const Slice& slice, const MmvdIdx candIdx, Mv deltaMv[NUM_REF_PIC_LIST_01]) const
{
  const int mvdBaseIdx = candIdx.pos.baseIdx;
  const int mvdStep = candIdx.pos.step;
  const int mvdPosition = candIdx.pos.position;

  int offset = 1 << (mvdStep + MV_FRACTIONAL_BITS_DIFF);
  if (slice.getPicHeader()->getDisFracMMVD())
  {
    offset <<= 2;
  }
  const int refList0 = mmvdBaseMv[mvdBaseIdx][REF_PIC_LIST_0].refIdx;
  const int refList1 = mmvdBaseMv[mvdBaseIdx][REF_PIC_LIST_1].refIdx;

  const Mv dMvTable[4] = { Mv(offset,0), Mv(-offset,0), Mv(0, offset), Mv(0, -offset) };
  if ((refList0 != -1) && (refList1 != -1))
  {
    const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
    const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
    const int currPoc = slice.getPOC();
    deltaMv[0] = dMvTable[mvdPosition];

    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      deltaMv[1] = deltaMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      deltaMv[1] = deltaMv[0];
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          deltaMv[0] = deltaMv[1];
        }
        else
        {
          deltaMv[0].set(-1 * deltaMv[1].getHor(), -1 * deltaMv[1].getVer());
        }
      }
      else
      {
        deltaMv[0] = deltaMv[1].getScaledMv(scale);
      }
    }
    else
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          deltaMv[1] = deltaMv[0];
        }
        else
        {
          deltaMv[1].set(-1 * deltaMv[0].getHor(), -1 * deltaMv[0].getVer());
        }
      }
      else
      {
        deltaMv[1] = deltaMv[0].getScaledMv(scale);
      }
    }
  }
  else if (refList0 != -1)
  {
    deltaMv[0] = dMvTable[mvdPosition];
  }
  else if (refList1 != -1)
  {
    deltaMv[1] = dMvTable[mvdPosition];
  }
}

void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit &pu, const MmvdIdx candIdx)
{
  Mv tempMv[NUM_REF_PIC_LIST_01];

#if GDR_ENABLED
  const CodingStructure &cs = *pu.cs;
  const bool             isEncodeGdrClean =
    cs.sps->getGDREnabledFlag() && cs.pcv->isEncoder
    && ((cs.picHeader->getInGdrInterval() && cs.isClean(pu.Y().topRight(), ChannelType::LUMA))
        || (cs.picHeader->getNumVerVirtualBoundaries() == 0));
#endif

  getMmvdDeltaMv(*pu.cs->slice, candIdx, tempMv);
  const int mvdBaseIdx  = candIdx.pos.baseIdx;

  const int refList0 = mmvdBaseMv[mvdBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[mvdBaseIdx][1].refIdx;

  if ((refList0 != -1) && (refList1 != -1))
  {
    pu.interDir = 3;
    pu.mv[REF_PIC_LIST_0]     = mmvdBaseMv[mvdBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1]     = mmvdBaseMv[mvdBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      Mv mv0 = pu.mv[REF_PIC_LIST_0];
      Mv mv1 = pu.mv[REF_PIC_LIST_1];

      int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
      int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

      mmvdValid[mvdBaseIdx][0] = cs.isClean(pu.Y().topRight(), mv0, REF_PIC_LIST_0, refIdx0);
      mmvdValid[mvdBaseIdx][1] = cs.isClean(pu.Y().topRight(), mv1, REF_PIC_LIST_1, refIdx1);

      pu.mvSolid[REF_PIC_LIST_0] = mmvdSolid[mvdBaseIdx][0];
      pu.mvSolid[REF_PIC_LIST_1] = mmvdSolid[mvdBaseIdx][1];

      pu.mvValid[REF_PIC_LIST_0] = mmvdValid[mvdBaseIdx][0];
      pu.mvValid[REF_PIC_LIST_1] = mmvdValid[mvdBaseIdx][1];
    }
#endif
  }
  else if (refList0 != -1)
  {
    pu.interDir = 1;
    pu.mv[REF_PIC_LIST_0]     = mmvdBaseMv[mvdBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;

#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      Mv mv0 = pu.mv[REF_PIC_LIST_0];
      //Mv mv1 = pu.mv[REF_PIC_LIST_1];

      int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
      //int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

      pu.mvSolid[REF_PIC_LIST_0] = mmvdSolid[mvdBaseIdx][0];
      pu.mvSolid[REF_PIC_LIST_1] = true;

      mmvdValid[mvdBaseIdx][0] = cs.isClean(pu.Y().topRight(), mv0, REF_PIC_LIST_0, refIdx0);
      mmvdValid[mvdBaseIdx][1] = true;

      pu.mvValid[REF_PIC_LIST_0] = mmvdValid[mvdBaseIdx][0];
      pu.mvValid[REF_PIC_LIST_1] = true;
    }
#endif
  }
  else if (refList1 != -1)
  {
    pu.interDir = 2;
    pu.mv[REF_PIC_LIST_0] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_0] = -1;
    pu.mv[REF_PIC_LIST_1]     = mmvdBaseMv[mvdBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      // Mv mv0 = pu.mv[REF_PIC_LIST_0];
      Mv mv1 = pu.mv[REF_PIC_LIST_1];

      // int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
      int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

      mmvdValid[mvdBaseIdx][0] = true;
      mmvdValid[mvdBaseIdx][1] = cs.isClean(pu.Y().topRight(), mv1, REF_PIC_LIST_1, refIdx1);

      pu.mvSolid[REF_PIC_LIST_0] = true;
      pu.mvSolid[REF_PIC_LIST_1] = mmvdSolid[mvdBaseIdx][1];

      pu.mvValid[REF_PIC_LIST_0] = true;
      pu.mvValid[REF_PIC_LIST_1] = mmvdValid[mvdBaseIdx][1];
    }
#endif
  }

  pu.mmvdMergeFlag    = true;
  pu.mmvdMergeIdx     = candIdx;
  pu.mergeFlag        = true;
  pu.regularMergeFlag = true;
  pu.mergeIdx         = candIdx.val;
  pu.mergeType        = MergeType::DEFAULT_N;

  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.cu->imv                = mmvdUseAltHpelIf[mvdBaseIdx] ? IMV_HPEL : 0;

  pu.cu->bcwIdx = (interDirNeighbours[mvdBaseIdx] == 3) ? bcwIdx[mvdBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      pu.mv[refList].clipToStorageBitDepth();
    }
  }

  PU::restrictBiPredMergeCandsOne(pu);
}

unsigned DeriveCtx::CtxMipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, ChannelType::LUMA);
  ctxId = (cuLeft && cuLeft->mipFlag) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, ChannelType::LUMA);
  ctxId += (cuAbove && cuAbove->mipFlag) ? 1 : 0;

  ctxId  = (cu.lwidth() > 2*cu.lheight() || cu.lheight() > 2*cu.lwidth()) ? 3 : ctxId;

  return ctxId;
}

unsigned DeriveCtx::CtxPltCopyFlag( const unsigned prevRunType, const unsigned dist )
{
  uint8_t *ucCtxLut = (prevRunType == PLT_RUN_INDEX) ? g_paletteRunLeftLut : g_paletteRunTopLut;
  if ( dist <= RUN_IDX_THRE )
  {
    return ucCtxLut[dist];
  }
  else
  {
    return ucCtxLut[RUN_IDX_THRE];
  }
}
