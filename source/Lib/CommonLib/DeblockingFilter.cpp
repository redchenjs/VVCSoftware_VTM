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

/** \file     DeblockingFilter.cpp
    \brief    deblocking filter
*/

#include "DeblockingFilter.h"
#include "Slice.h"
#include "Mv.h"
#include "Unit.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define DEBLOCK_SMALLEST_BLOCK  8
constexpr int DEFAULT_INTRA_TC_OFFSET = 2;   // Default intra TC offset

constexpr DeblockingFilter::FilterLenPair DeblockingFilter::DEFAULT_FL2;

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint16_t DeblockingFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   3,   4,   4,   4,
  4,  5,  5,  5,  5,  7,  7,  8,  9,  10,  10,  11,  13,  14,  15,  17,  19,  21,  24,  25,  29,  33,
  36, 41, 45, 51, 57, 64, 71, 80, 89, 100, 112, 125, 141, 157, 177, 198, 222, 250, 280, 314, 352, 395
};

const uint8_t DeblockingFilter::sm_betaTable[MAX_QP + 1] = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                                                       6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 24,
                                                       26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56,
                                                       58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88 };

inline static uint32_t getRasterIdx(const Position& pos, const PreCalcValues& pcv)
{
  return ( ( pos.x & pcv.maxCUWidthMask ) >> pcv.minCUWidthLog2 ) + ( ( pos.y & pcv.maxCUHeightMask ) >> pcv.minCUHeightLog2 ) * pcv.partsInCtuWidth;
}

// ====================================================================================================================
// utility functions
// ====================================================================================================================
static bool isNeighbourAvailable(const CodingUnit &cu, const CodingUnit &cu2, const PPS &pps)
{
  return (pps.getLoopFilterAcrossSlicesEnabledFlag() || CU::isSameSlice(cu, cu2))
         && (pps.getLoopFilterAcrossTilesEnabledFlag() || CU::isSameTile(cu, cu2))
         && ((pps.getSubPicFromCU(cu).getloopFilterAcrossEnabledFlag()
              && pps.getSubPicFromCU(cu2).getloopFilterAcrossEnabledFlag())
             || CU::isSameSubPic(cu, cu2));
}

void DeblockingFilter::clearFilterLengthAndTransformEdge()
{
  std::fill_n(m_maxFilterLen[0][0], sizeof(m_maxFilterLen) / sizeof(FilterLenPair),
              FilterLenPair{ FilterLen::_1, FilterLen::_1 });
  std::fill_n(m_transformEdge[0][0], sizeof(m_transformEdge) / sizeof(bool), false);
}

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

DeblockingFilter::DeblockingFilter()
{
}

DeblockingFilter::~DeblockingFilter()
{
  m_encPicYuvBuffer.destroy();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void DeblockingFilter::create(const unsigned maxCUDepth)
{
  destroy();
  const auto numPartitions = size_t(1) << (2 * maxCUDepth);
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_edgeStrengths[edgeDir].resize(numPartitions);
  }
  m_enc = false;
}

void DeblockingFilter::initEncPicYuvBuffer(ChromaFormat chromaFormat, const Size &size, const unsigned maxCUSize)
{
  const Area a = Area(Position(), size);
  m_encPicYuvBuffer.destroy();
  m_encPicYuvBuffer.create(chromaFormat, a, maxCUSize, 7);
}

void DeblockingFilter::destroy()
{
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_edgeStrengths[edgeDir].clear();
  }
}

void DeblockingFilter::deblockingFilterPic(CodingStructure &cs)
{
  const PreCalcValues& pcv = *cs.pcv;
  m_shiftHor = ::getComponentScaleX( COMPONENT_Cb, cs.pcv->chrFormat );
  m_shiftVer = ::getComponentScaleY( COMPONENT_Cb, cs.pcv->chrFormat );

  DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", cs.slice->getPOC() ) ) );
#if ENABLE_TRACING
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );
    }
  }
#endif
#if GREEN_METADATA_SEI_ENABLED
  FeatureCounterStruct tempFeatureCounter;
#endif

  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      resetBsAndEdgeFilter(EDGE_VER);
      clearFilterLengthAndTransformEdge();
      m_ctuXLumaSamples = x << pcv.maxCUWidthLog2;
      m_ctuYLumaSamples = y << pcv.maxCUHeightLog2;

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      CodingUnit *firstCU = cs.getCU(ctuArea.lumaPos(), CHANNEL_TYPE_LUMA);
      cs.slice = firstCU->slice;

      // CU-based deblocking
      for (auto &currCU: cs.traverseCUs(CS::getArea(cs, ctuArea, CHANNEL_TYPE_LUMA), CHANNEL_TYPE_LUMA))
      {
#if GREEN_METADATA_SEI_ENABLED
        currCU.m_featureCounter.resetBoundaryStrengths();
#endif
        xDeblockCU( currCU, EDGE_VER );
#if GREEN_METADATA_SEI_ENABLED
        tempFeatureCounter.addBoundaryStrengths(currCU.m_featureCounter);
#endif
      }

      if( CS::isDualITree( cs ) )
      {
        resetBsAndEdgeFilter(EDGE_VER);
        clearFilterLengthAndTransformEdge();

        for (auto &currCU: cs.traverseCUs(CS::getArea(cs, ctuArea, CHANNEL_TYPE_CHROMA), CHANNEL_TYPE_CHROMA))
        {
#if GREEN_METADATA_SEI_ENABLED
          currCU.m_featureCounter.resetBoundaryStrengths();
#endif
          xDeblockCU( currCU, EDGE_VER );
#if GREEN_METADATA_SEI_ENABLED
          tempFeatureCounter.addBoundaryStrengths(currCU.m_featureCounter);
#endif
        }
      }
    }
  }

  // Vertical filtering
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      resetBsAndEdgeFilter(EDGE_HOR);
      clearFilterLengthAndTransformEdge();
      m_ctuXLumaSamples = x << pcv.maxCUWidthLog2;
      m_ctuYLumaSamples = y << pcv.maxCUHeightLog2;

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      CodingUnit *firstCU = cs.getCU(ctuArea.lumaPos(), CHANNEL_TYPE_LUMA);
      cs.slice = firstCU->slice;

      // CU-based deblocking
      for (auto &currCU: cs.traverseCUs(CS::getArea(cs, ctuArea, CHANNEL_TYPE_LUMA), CHANNEL_TYPE_LUMA))
      {
#if GREEN_METADATA_SEI_ENABLED
        currCU.m_featureCounter.resetBoundaryStrengths();
#endif
        xDeblockCU( currCU, EDGE_HOR );
#if GREEN_METADATA_SEI_ENABLED
        tempFeatureCounter.addBoundaryStrengths(currCU.m_featureCounter);
#endif
      }

      if( CS::isDualITree( cs ) )
      {
        resetBsAndEdgeFilter(EDGE_HOR);
        clearFilterLengthAndTransformEdge();

        for (auto &currCU: cs.traverseCUs(CS::getArea(cs, ctuArea, CHANNEL_TYPE_CHROMA), CHANNEL_TYPE_CHROMA))
        {
#if GREEN_METADATA_SEI_ENABLED
          currCU.m_featureCounter.resetBoundaryStrengths();
#endif
          xDeblockCU( currCU, EDGE_HOR );
#if GREEN_METADATA_SEI_ENABLED
          tempFeatureCounter.addBoundaryStrengths(currCU.m_featureCounter);
#endif
        }
      }
    }
  }

#if GREEN_METADATA_SEI_ENABLED
  cs.m_featureCounter.addBoundaryStrengths(tempFeatureCounter);
#endif
  DTRACE_PIC_COMP(D_REC_CB_LUMA_LF,   cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "DeblockingFilter" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
}

void DeblockingFilter::resetBsAndEdgeFilter(const int edgeDir)
{
  std::fill_n(m_edgeStrengths[edgeDir].data(), m_edgeStrengths[edgeDir].size(), EdgeStrengths());
}

void DeblockingFilter::resetFilterLengths()
{
  resetBsAndEdgeFilter(EDGE_VER);
  resetBsAndEdgeFilter(EDGE_HOR);
  clearFilterLengthAndTransformEdge();
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void DeblockingFilter::xDeblockCU( CodingUnit& cu, const DeblockEdgeDir edgeDir )
{
  const PreCalcValues& pcv = *cu.cs->pcv;
  const Area area          = cu.Y().valid() ? cu.Y() : Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) );

  bool horEdgeFilter = false, verEdgeFilter = false;
  int  numHorVirBndry = 0, numVerVirBndry = 0;
  int  horVirBndryPos[] = { 0, 0, 0 };
  int  verVirBndryPos[] = { 0, 0, 0 };

  bool isCuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries( area.x, area.y, area.width, area.height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cu.cs->picHeader );

  xSetDeblockingFilterParam( cu );
  static_vector<int, 2*MAX_CU_SIZE> edgeIdx;
  edgeIdx.clear();

  if (m_enc)
  {
    m_shiftHor = ::getComponentScaleX(COMPONENT_Cb, cu.chromaFormat);
    m_shiftVer = ::getComponentScaleY(COMPONENT_Cb, cu.chromaFormat);
    int x, y;
    if (cu.Y().valid())
    {
      x = cu.block(COMPONENT_Y).x;
      y = cu.block(COMPONENT_Y).y;
    }
    else
    {
      x = cu.block(COMPONENT_Cb).x << m_shiftHor;
      y = cu.block(COMPONENT_Cb).y << m_shiftVer;
    }
    m_ctuXLumaSamples = x & ~(cu.slice->getSPS()->getMaxCUWidth()  - 1);
    m_ctuYLumaSamples = y & ~(cu.slice->getSPS()->getMaxCUHeight() - 1);
  }

  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    const Area& areaTu = cu.Y().valid() ? currTU.block( COMPONENT_Y ) : Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, currTU.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, currTU.blocks[cu.chType].size() ) );
    const bool  xOff   = currTU.blocks[cu.chType].x != cu.blocks[cu.chType].x;
    const bool  yOff   = currTU.blocks[cu.chType].y != cu.blocks[cu.chType].y;

    verEdgeFilter = xOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.leftEdge;
    horEdgeFilter = yOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.topEdge;

    if ((edgeDir == EDGE_HOR && areaTu.y % 4 != 0) || (edgeDir == EDGE_VER && areaTu.x % 4 != 0))
    {
      if (cu.chromaFormat != CHROMA_400 && currTU.block(COMPONENT_Cb).valid())
      {
        if ((edgeDir == EDGE_HOR && currTU.block(COMPONENT_Cb).y % 4 == 0)
            || (edgeDir == EDGE_VER && currTU.block(COMPONENT_Cb).x % 4 == 0))
        {
          // Set max filter length for chroma in narrow/short CUs that use ISP mode
          xSetMaxFilterLengthPQFromTransformSizes(edgeDir, cu, currTU, COMPONENT_Cb);
        }
      }
      continue;
    }

    if( isCuCrossedByVirtualBoundaries )
    {
      xDeriveEdgefilterParam( areaTu.x, areaTu.y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
    }
    xSetEdgefilterMultiple(cu, EDGE_VER, areaTu, verEdgeFilter, true);
    xSetEdgefilterMultiple(cu, EDGE_HOR, areaTu, horEdgeFilter, true);
    xSetMaxFilterLengthPQFromTransformSizes(edgeDir, cu, currTU, COMPONENT_Y);
    if( cu.Y().valid() )
    {
      edgeIdx.push_back( ( edgeDir == EDGE_HOR ) ? ( currTU.blocks[cu.chType].y - cu.blocks[cu.chType].y ) / 4 : ( currTU.blocks[cu.chType].x - cu.blocks[cu.chType].x ) / 4 );
    }
    else
    {
      edgeIdx.push_back( ( edgeDir == EDGE_HOR ) ? (( currTU.blocks[cu.chType].y - cu.blocks[cu.chType].y ) << ::getComponentScaleY(COMPONENT_Cb, cu.chromaFormat))  / 4 : (( currTU.blocks[cu.chType].x - cu.blocks[cu.chType].x ) << ::getComponentScaleX(COMPONENT_Cb, cu.chromaFormat)) / 4 );
    }
  }

  bool mvSubBlocks = false;
  int subBlockSize = 8;
  for( auto &currPU : CU::traversePUs( cu ) )
  {
    const Area& areaPu = cu.Y().valid() ? currPU.block( COMPONENT_Y ) : area;
    const bool xOff    = currPU.blocks[cu.chType].x != cu.blocks[cu.chType].x;
    const bool yOff    = currPU.blocks[cu.chType].y != cu.blocks[cu.chType].y;

    verEdgeFilter = xOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.leftEdge;
    horEdgeFilter = yOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.topEdge;

    if( isCuCrossedByVirtualBoundaries )
    {
      xDeriveEdgefilterParam( areaPu.x, areaPu.y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
    }

    xSetEdgefilterMultiple(cu, EDGE_VER, areaPu, verEdgeFilter, false);
    xSetEdgefilterMultiple(cu, EDGE_HOR, areaPu, horEdgeFilter, false);
    edgeIdx.push_back( ( edgeDir == EDGE_HOR ) ? ( currPU.blocks[cu.chType].y - cu.blocks[cu.chType].y ) / 4 : ( currPU.blocks[cu.chType].x - cu.blocks[cu.chType].x ) / 4 );

    if ((currPU.mergeFlag && (currPU.mergeType == MRG_TYPE_SUBPU_ATMVP)) || cu.affine)
    {
      mvSubBlocks = true;
      if (edgeDir == EDGE_HOR)
      {
        for (uint32_t off = subBlockSize; off < areaPu.height; off += subBlockSize)
        {
          const Area mvBlockH(cu.Y().x, cu.Y().y + off, cu.Y().width, pcv.minCUHeight);
          horEdgeFilter = m_stLFCUParam.internalEdge;
          if( isCuCrossedByVirtualBoundaries )
          {
            xDeriveEdgefilterParam( mvBlockH.x, mvBlockH.y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
          }

          xSetEdgefilterMultiple(cu, EDGE_HOR, mvBlockH, horEdgeFilter, false);
          edgeIdx.push_back( ( currPU.blocks[cu.chType].y + off - cu.blocks[cu.chType].y ) / 4 );
        }
      }
      else
      {
        for (uint32_t off = subBlockSize; off < areaPu.width; off += subBlockSize)
        {
          const Area mvBlockV(cu.Y().x + off, cu.Y().y, pcv.minCUWidth, cu.Y().height);
          verEdgeFilter = m_stLFCUParam.internalEdge;
          if( isCuCrossedByVirtualBoundaries )
          {
            xDeriveEdgefilterParam( mvBlockV.x, mvBlockV.y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
          }

          xSetEdgefilterMultiple(cu, EDGE_VER, mvBlockV, verEdgeFilter, false);
          edgeIdx.push_back( ( currPU.blocks[cu.chType].x + off - cu.blocks[cu.chType].x ) / 4 );
        }
      }
    }

    xSetMaxFilterLengthPQForCodingSubBlocks( edgeDir, cu, currPU, mvSubBlocks, subBlockSize, areaPu );
  }

  const unsigned pelsInPart = pcv.minCUWidth;

  for (int y = 0; y < area.height; y += pelsInPart)
  {
    for (int x = 0; x < area.width; x += pelsInPart)
    {
      const Position localPos  { area.x + x, area.y + y };
      const unsigned rasterIdx = getRasterIdx( localPos, pcv );

      EdgeStrengths es = m_edgeStrengths[edgeDir][rasterIdx];
      if (es.hasEdge())
      {
        if(cu.treeType != TREE_C)
        {
          es |= xGetBoundaryStrengthSingle(cu, edgeDir, localPos, CHANNEL_TYPE_LUMA);
#if GREEN_METADATA_SEI_ENABLED
          const int bsY = es.getBoundaryStrength(COMPONENT_Y);
          cu.m_featureCounter.boundaryStrength[bsY]++;
          cu.m_featureCounter.boundaryStrengthPel[bsY] += pelsInPart;
#endif
        }
        if(cu.treeType != TREE_L && cu.chromaFormat != CHROMA_400 && cu.blocks[COMPONENT_Cb].valid())
        {
          es |= xGetBoundaryStrengthSingle(cu, edgeDir, localPos, CHANNEL_TYPE_CHROMA);
#if GREEN_METADATA_SEI_ENABLED
          const int bsCb = es.getBoundaryStrength(COMPONENT_Cb);
          const int bsCr = es.getBoundaryStrength(COMPONENT_Cr);
          cu.m_featureCounter.boundaryStrength[bsCb]++;
          cu.m_featureCounter.boundaryStrength[bsCr]++;
          cu.m_featureCounter.boundaryStrengthPel[bsCb] += pelsInPart;
          cu.m_featureCounter.boundaryStrengthPel[bsCr] += pelsInPart;
#endif
        }
        m_edgeStrengths[edgeDir][rasterIdx] = es;
      }
    }
  }


  std::sort( edgeIdx.begin(), edgeIdx.end() );
  int prevEdgeIdx = -1;
  for ( const int& edge : edgeIdx )
  {
    if ( edge == prevEdgeIdx ) // skip duplicate edgeIdx marked by both transform and coding subblock processes
    {
      continue;
    }
    prevEdgeIdx = edge;

    if ( cu.blocks[COMPONENT_Y].valid() )
    {
      xEdgeFilterLuma( cu, edgeDir, edge );
    }

    if ( pcv.chrFormat != CHROMA_400 && cu.blocks[COMPONENT_Cb].valid() )
    {
      if ( !cu.ispMode || edge == 0 )
      {
        xEdgeFilterChroma( cu, edgeDir, edge );
      }
    }
  }
}

inline bool DeblockingFilter::isCrossedByVirtualBoundaries(const int xPos, const int yPos, const int width, const int height, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], const PicHeader* picHeader )
{
  numHorVirBndry = 0; numVerVirBndry = 0;
  if( picHeader->getVirtualBoundariesPresentFlag() )
  {
    for (int i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++)
    {
      if (yPos <= picHeader->getVirtualBoundariesPosY(i) && picHeader->getVirtualBoundariesPosY(i) < yPos + height)
      {
        horVirBndryPos[numHorVirBndry++] = picHeader->getVirtualBoundariesPosY(i);
      }
    }
    for (int i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++)
    {
      if (xPos <= picHeader->getVirtualBoundariesPosX(i) && picHeader->getVirtualBoundariesPosX(i) < xPos + width)
      {
        verVirBndryPos[numVerVirBndry++] = picHeader->getVirtualBoundariesPosX(i);
      }
    }
  }
  return numHorVirBndry > 0 || numVerVirBndry > 0;
}

inline void DeblockingFilter::xDeriveEdgefilterParam( const int xPos, const int yPos, const int numVerVirBndry, const int numHorVirBndry, const int verVirBndryPos[], const int horVirBndryPos[], bool &verEdgeFilter, bool &horEdgeFilter )
{
  for (int i = 0; i < numVerVirBndry; i++)
  {
    if (verVirBndryPos[i] == xPos)
    {
      verEdgeFilter = false;
      break;
    }
  }

  for (int i = 0; i < numHorVirBndry; i++)
  {
    if (horVirBndryPos[i] == yPos)
    {
      horEdgeFilter = false;
      break;
    }
  }
}

void DeblockingFilter::xSetMaxFilterLengthPQFromTransformSizes(const DeblockEdgeDir edgeDir, const CodingUnit &cu,
                                                               const TransformUnit &currTU, const int firstComponent)
{
  const TransformUnit& tuQ = currTU;

  if ( edgeDir == EDGE_HOR )
  {
    for (int cIdx = firstComponent; cIdx < ::getNumberValidComponents(tuQ.chromaFormat); cIdx++)   // per component
    {
      const ComponentID comp = ComponentID(cIdx);
      if (comp > COMPONENT_Cb)
      {
        continue;
      }
      const ChannelType ch   = toChannelType(comp);

      const int shiftHor = isLuma(ch) ? 0 : m_shiftHor;
      const int shiftVer = isLuma(ch) ? 0 : m_shiftVer;
      const int ctuXOff      = currTU.block(comp).x - ( m_ctuXLumaSamples >> shiftHor ); // x offset from left edge of CTU in respective channel sample units
      const int ctuYOff      = currTU.block(comp).y - ( m_ctuYLumaSamples >> shiftVer ); // y offset from top edge of CTU in respective channel sample units
      const int minCUWidth   = cu.cs->pcv->minCUWidth >> shiftHor;
      const int gridShiftHor = LOG_GRID_SIZE - shiftHor;
      const int gridShiftVer = LOG_GRID_SIZE - shiftVer;

      if ( currTU.block(comp).valid() && ( ( currTU.block(comp).y == cu.block(comp).y ) ? m_stLFCUParam.topEdge : m_stLFCUParam.internalEdge ) ) // Edge deblocking needs to be recomputed since ISP contains whole CU chroma transforms in last TU of the CU
      {
        for ( int x = 0; x < currTU.blocks[cIdx].width; x += minCUWidth )
        {
          const Position  posQ     = Position( currTU.blocks[ch].x + x, currTU.blocks[ch].y );
          const Position  posP     = posQ.offset( 0, -1 );
          const int sizeQSide      = tuQ.block(comp).height;
          const TransformUnit& tuP = *cu.cs->getTU( posP, ch );
          const int sizePSide      = tuP.block(comp).height;
          m_transformEdge[ch][(ctuXOff + x) >> gridShiftHor][ctuYOff >> gridShiftVer] = true;

          auto &maxFilterLen = m_maxFilterLen[ch][(ctuXOff + x) >> gridShiftHor][ctuYOff >> gridShiftVer];

          if (isLuma(ch))
          {
            if (sizePSide <= 4 || sizeQSide <= 4)
            {
              maxFilterLen.q = FilterLen::_1;
              maxFilterLen.p = FilterLen::_1;
            }
            else
            {
              maxFilterLen.q = sizeQSide >= 32 ? FilterLen::_7 : FilterLen::_3;
              maxFilterLen.p = sizePSide >= 32 ? FilterLen::_7 : FilterLen::_3;
            }
          }
          else
          {
            maxFilterLen.q = sizeQSide >= 8 && sizePSide >= 8 ? FilterLen::_3 : FilterLen::_1;
            maxFilterLen.p = sizeQSide >= 8 && sizePSide >= 8 ? FilterLen::_3 : FilterLen::_1;
          }
        }
      }
    }
  }
  if ( edgeDir == EDGE_VER )
  {
    for ( int cIdx = firstComponent; cIdx < ::getNumberValidComponents(tuQ.chromaFormat); cIdx++ ) // per component
    {
      const ComponentID comp = ComponentID(cIdx);
      if (comp > COMPONENT_Cb)
      {
        continue;
      }
      const ChannelType ch   = toChannelType(comp);

      const int shiftHor = isLuma(ch) ? 0 : m_shiftHor;
      const int shiftVer = isLuma(ch) ? 0 : m_shiftVer;
      const int ctuXOff      = currTU.block(comp).x - ( m_ctuXLumaSamples >> shiftHor ); // x offset from left edge of CTU in respective channel sample units
      const int ctuYOff      = currTU.block(comp).y - ( m_ctuYLumaSamples >> shiftVer ); // y offset from top edge of CTU in respective channel sample units
      const int minCUHeight  = cu.cs->pcv->minCUHeight >> shiftVer;
      const int gridShiftHor = LOG_GRID_SIZE - shiftHor;
      const int gridShiftVer = LOG_GRID_SIZE - shiftVer;

      if ( currTU.block(comp).valid() && ( ( currTU.block(comp).x == cu.block(comp).x ) ? m_stLFCUParam.leftEdge : m_stLFCUParam.internalEdge ) ) // Edge deblocking needs to be recomputed since ISP contains whole CU chroma transforms in last TU of the CU
      {
        for ( int y = 0; y < currTU.blocks[cIdx].height; y += minCUHeight )
        {
          const Position  posQ     = Position( currTU.blocks[ch].x, currTU.blocks[ch].y + y );
          const Position  posP     = posQ.offset( -1, 0 );
          const int sizeQSide      = tuQ.block(comp).width;
          const TransformUnit& tuP = *cu.cs->getTU( posP, ch );
          const int sizePSide      = tuP.block(comp).width;
          m_transformEdge[ch][ctuXOff >> gridShiftHor][(ctuYOff + y) >> gridShiftVer] = true;

          auto &maxFilterLen = m_maxFilterLen[ch][ctuXOff >> gridShiftHor][(ctuYOff + y) >> gridShiftVer];

          if (isLuma(ch))
          {
            if (sizePSide <= 4 || sizeQSide <= 4)
            {
              maxFilterLen.q = FilterLen::_1;
              maxFilterLen.p = FilterLen::_1;
            }
            else
            {
              maxFilterLen.q = sizeQSide >= 32 ? FilterLen::_7 : FilterLen::_3;
              maxFilterLen.p = sizePSide >= 32 ? FilterLen::_7 : FilterLen::_3;
            }
          }
          else
          {
            maxFilterLen.q = sizeQSide >= 8 && sizePSide >= 8 ? FilterLen::_3 : FilterLen::_1;
            maxFilterLen.p = sizeQSide >= 8 && sizePSide >= 8 ? FilterLen::_3 : FilterLen::_1;
          }
        }
      }
    }
  }
}

void DeblockingFilter::xSetMaxFilterLengthPQForCodingSubBlocks( const DeblockEdgeDir edgeDir, const CodingUnit& cu, const PredictionUnit& currPU, const bool& mvSubBlocks, const int& subBlockSize, const Area& areaPu )
{
  if ( mvSubBlocks && currPU.Y().valid() )
  {
    const ComponentID comp         = COMPONENT_Y;
    const ChannelType ch           = toChannelType(comp);
    const int ctuYOff      = currPU.block(comp).y - m_ctuYLumaSamples; // y offset from top edge of CTU in luma samples
    const int ctuXOff      = currPU.block(comp).x - m_ctuXLumaSamples; // x offset from left edge of CTU in luma samples
    const int minCUWidth   = cu.cs->pcv->minCUWidth;
    const int minCUHeight  = cu.cs->pcv->minCUHeight;

    if ( edgeDir == EDGE_HOR )
    {
      for ( int y = 0; y < areaPu.height; y += subBlockSize )
      {
        for ( int x = 0; x < areaPu.width; x += minCUWidth )
        {
          const int xg = (ctuXOff + x) >> LOG_GRID_SIZE;
          const int yg = (ctuYOff + y) >> LOG_GRID_SIZE;

          auto &maxFilterLen = m_maxFilterLen[ch][xg][yg];

          if (m_transformEdge[ch][xg][yg])
          {
            maxFilterLen.q = std::min(maxFilterLen.q, FilterLen::_5);
            if (y > 0)
            {
              maxFilterLen.p = std::min(maxFilterLen.p, FilterLen::_5);
            }
          }
          // adjacent to transform edge  +/- 4
          else if (y > 0
                   && (y + 4 >= areaPu.height || m_transformEdge[ch][xg][yg - 1] || m_transformEdge[ch][xg][yg + 1]))
          {
            maxFilterLen.q = FilterLen::_1;
            maxFilterLen.p = FilterLen::_1;
          }
          // adjacent to transform edge on 8x8 grid
          else if (y > 0
                   && (y == 8 || y + 8 >= areaPu.height || m_transformEdge[ch][xg][yg - 2]
                       || m_transformEdge[ch][xg][yg + 2]))
          {
            maxFilterLen.q = FilterLen::_2;
            maxFilterLen.p = FilterLen::_2;
          }
          else
          {
            maxFilterLen.q = FilterLen::_3;
            maxFilterLen.p = FilterLen::_3;
          }
        }
      }
    }
    else // edgeDir == EDGE_VER
    {
      for ( int x = 0; x < areaPu.width; x += subBlockSize )
      {
        for ( int y = 0; y < areaPu.height; y += minCUHeight )
        {
          const int xg = (ctuXOff + x) >> LOG_GRID_SIZE;
          const int yg = (ctuYOff + y) >> LOG_GRID_SIZE;

          auto &maxFilterLen = m_maxFilterLen[ch][xg][yg];

          if (m_transformEdge[ch][xg][yg])
          {
            maxFilterLen.q = std::min(maxFilterLen.q, FilterLen::_5);
            if (x > 0)
            {
              maxFilterLen.p = std::min(maxFilterLen.p, FilterLen::_5);
            }
          }
          // adjacent to transform edge +/- 4
          else if (x > 0
                   && (x + 4 >= areaPu.width || m_transformEdge[ch][xg - 1][yg] || m_transformEdge[ch][xg + 1][yg]))
          {
            maxFilterLen.q = FilterLen::_1;
            maxFilterLen.p = FilterLen::_1;
          }
          // adjacent to transform edge on 8x8 grid
          else if (x > 0
                   && (x == 8 || x + 8 >= areaPu.width || m_transformEdge[ch][xg - 2][yg]
                       || m_transformEdge[ch][xg + 2][yg]))
          {
            maxFilterLen.q = FilterLen::_2;
            maxFilterLen.p = FilterLen::_2;
          }
          else
          {
            maxFilterLen.q = FilterLen::_3;
            maxFilterLen.p = FilterLen::_3;
          }
        }
      }
    }
  }
}

void DeblockingFilter::xSetEdgefilterMultiple(const CodingUnit &cu, const DeblockEdgeDir edgeDir, const Area &area,
                                              const bool value, const bool isTransEdge)
{
  const PreCalcValues& pcv = *cu.cs->pcv;

  const unsigned add     = (edgeDir == EDGE_VER) ? pcv.partsInCtuWidth : 1;
  const unsigned numElem = (edgeDir == EDGE_VER) ? (area.height / pcv.minCUHeight) : (area.width / pcv.minCUWidth);
  unsigned       bsIdx   = getRasterIdx(area, pcv);

  for (int i = 0; i < numElem; i++)
  {
    if (isTransEdge)
    {
      m_edgeStrengths[edgeDir][bsIdx].setTransEdge(value);
    }
    else
    {
      m_edgeStrengths[edgeDir][bsIdx].setPredEdge(value);
    }

    bsIdx += add;
  }
}

void DeblockingFilter::xSetDeblockingFilterParam( const CodingUnit& cu )
{
  const Slice& slice = *cu.slice;
  const PPS&   pps   = *cu.cs->pps;

  if( slice.getDeblockingFilterDisable() )
  {
    m_stLFCUParam.leftEdge = m_stLFCUParam.topEdge = m_stLFCUParam.internalEdge = false;
    return;
  }

  const Position& pos = cu.blocks[cu.chType].pos();

  m_stLFCUParam.internalEdge = true;
  m_stLFCUParam.leftEdge     = false;
  m_stLFCUParam.topEdge      = false;

  if (pos.x > 0)
  {
    CodingUnit *neighbourCu = cu.cs->getCU(pos.offset(-1, 0), cu.chType);

    m_stLFCUParam.leftEdge = isNeighbourAvailable(cu, *neighbourCu, pps);
  }
  if (pos.y > 0)
  {
    CodingUnit *neighbourCu = cu.cs->getCU(pos.offset(0, -1), cu.chType);

    m_stLFCUParam.topEdge = isNeighbourAvailable(cu, *neighbourCu, pps);
  }
}

DeblockingFilter::EdgeStrengths DeblockingFilter::xGetBoundaryStrengthSingle(const CodingUnit    &cu,
                                                                             const DeblockEdgeDir edgeDir,
                                                                             const Position      &localPos,
                                                                             const ChannelType    chType) const
{
  int shiftHor = cu.Y().valid() ? 0 : ::getComponentScaleX(COMPONENT_Cb, cu.firstPU->chromaFormat);
  int shiftVer = cu.Y().valid() ? 0 : ::getComponentScaleY(COMPONENT_Cb, cu.firstPU->chromaFormat);
  const Position& posQ = Position{ localPos.x >> shiftHor,  localPos.y >> shiftVer };
  const Position  posP  = ( edgeDir == EDGE_VER ) ? posQ.offset( -1, 0 ) : posQ.offset( 0, -1 );

  const CodingUnit& cuQ = cu;
  const CodingUnit& cuP = (chType == CHANNEL_TYPE_CHROMA && cuQ.chType == CHANNEL_TYPE_LUMA) ?
                          *cu.cs->getCU(recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA, posP), CHANNEL_TYPE_CHROMA) :
                          *cu.cs->getCU( posP, cu.chType );

  if (CU::isIntra(cuP) || CU::isIntra(cuQ))
  {
    if (isLuma(chType))
    {
      const int bsY =
        CU::isIntra(cuP) && cuP.bdpcmMode != BdpcmMode::NONE && CU::isIntra(cuQ) && cuQ.bdpcmMode != BdpcmMode::NONE
          ? 0
          : 2;
      return EdgeStrengths().setBoundaryStrength(COMPONENT_Y, bsY);
    }
    else
    {
      const int bsC = CU::isIntra(cuP) && cuP.bdpcmModeChroma != BdpcmMode::NONE && CU::isIntra(cuQ)
                          && cuQ.bdpcmModeChroma != BdpcmMode::NONE
                        ? 0
                        : 2;
      return EdgeStrengths().setBoundaryStrength(COMPONENT_Cb, bsC).setBoundaryStrength(COMPONENT_Cr, bsC);
    }
  }

  const TransformUnit& tuQ = *cuQ.cs->getTU(posQ, cuQ.chType);
  const TransformUnit& tuP = (cuP.chType == CHANNEL_TYPE_CHROMA && cuQ.chType == CHANNEL_TYPE_LUMA) ?
                             *cuP.cs->getTU(recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA, posP), CHANNEL_TYPE_CHROMA) :
                             *cuP.cs->getTU(posP, cuQ.chType);

  const PreCalcValues& pcv = *cu.cs->pcv;
  const unsigned rasterIdx = getRasterIdx( Position{ localPos.x,  localPos.y }, pcv );
  if (m_edgeStrengths[edgeDir][rasterIdx].getTransEdge() && (cuP.firstPU->ciipFlag || cuQ.firstPU->ciipFlag))
  {
    if(chType == CHANNEL_TYPE_LUMA)
    {
      return EdgeStrengths().setBoundaryStrength(COMPONENT_Y, 2);
    }
    else
    {
      return EdgeStrengths().setBoundaryStrength(COMPONENT_Cb, 2).setBoundaryStrength(COMPONENT_Cr, 2);
    }
  }

  EdgeStrengths tmpBs;

  //-- Set BS for not Intra MB : BS = 2 or 1 or 0
  if(chType == CHANNEL_TYPE_LUMA)
  {
    // Y
    if (m_edgeStrengths[edgeDir][rasterIdx].getTransEdge()
        && (TU::getCbf(tuQ, COMPONENT_Y) || TU::getCbf(tuP, COMPONENT_Y)))
    {
      tmpBs.setBoundaryStrength(COMPONENT_Y, 1);
    }
  }
  else
  {
    if (pcv.chrFormat != CHROMA_400)
    {
      // U
      if (m_edgeStrengths[edgeDir][rasterIdx].getTransEdge()
          && (TU::getCbf(tuQ, COMPONENT_Cb) || TU::getCbf(tuP, COMPONENT_Cb) || tuQ.jointCbCr || tuP.jointCbCr))
      {
        tmpBs.setBoundaryStrength(COMPONENT_Cb, 1);
      }
      // V
      if (m_edgeStrengths[edgeDir][rasterIdx].getTransEdge()
          && (TU::getCbf(tuQ, COMPONENT_Cr) || TU::getCbf(tuP, COMPONENT_Cr) || tuQ.jointCbCr || tuP.jointCbCr))
      {
        tmpBs.setBoundaryStrength(COMPONENT_Cr, 1);
      }
    }
  }

  if (tmpBs.getBoundaryStrength(COMPONENT_Y) == 1 || !cu.Y().valid())
  {
    return tmpBs;
  }

  // and now the pred
  if (m_edgeStrengths[edgeDir][rasterIdx].getTransEdge() && !m_edgeStrengths[edgeDir][rasterIdx].getPredEdge())
  {
    return tmpBs;
  }
  if( chType == CHANNEL_TYPE_CHROMA )
  {
    return tmpBs;
  }
  if( cuP.predMode != cuQ.predMode && chType == CHANNEL_TYPE_LUMA )
  {
    return EdgeStrengths().setBoundaryStrength(COMPONENT_Y, 1);
  }

  const Position lumaPosQ = localPos;
  const Position lumaPosP = edgeDir == EDGE_VER ? lumaPosQ.offset(-1, 0) : lumaPosQ.offset(0, -1);

  const MotionInfo &miQ = cuQ.cs->getMotionInfo(lumaPosQ);
  const MotionInfo &miP = cuP.cs->getMotionInfo(lumaPosP);

  auto motionBreak = [](const Mv &a, const Mv &b) -> bool
  {
    constexpr int MVD_TH = 1 << MV_FRACTIONAL_BITS_INTERNAL >> 1;
    return abs(b.getHor() - a.getHor()) >= MVD_TH || abs(b.getVer() - a.getVer()) >= MVD_TH;
  };

  auto getPic = [](const CodingUnit &cu, const MotionInfo &mi, RefPicList l) -> const Picture *
  {
    return CU::isIBC(cu) ? (l == REF_PIC_LIST_0 ? cu.slice->getPic() : nullptr)
                         : (mi.refIdx[l] >= 0 ? cu.slice->getRefPic(l, mi.refIdx[l]) : nullptr);
  };

  bool d = true;   // by default assume there is discontinuity

  const Picture *refP0 = getPic(cuP, miP, REF_PIC_LIST_0);
  const Picture *refQ0 = getPic(cuQ, miQ, REF_PIC_LIST_0);

  if (cuQ.slice->isInterB() || cuP.slice->isInterB())
  {
    const Picture *refP1 = getPic(cuP, miP, REF_PIC_LIST_1);
    const Picture *refQ1 = getPic(cuQ, miQ, REF_PIC_LIST_1);

    if ((refP0 == refQ0 && refP1 == refQ1) || (refP0 == refQ1 && refP1 == refQ0))
    {
      const Mv mvP0 = miP.refIdx[REF_PIC_LIST_0] >= 0 ? miP.mv[REF_PIC_LIST_0] : Mv();
      const Mv mvP1 = miP.refIdx[REF_PIC_LIST_1] >= 0 ? miP.mv[REF_PIC_LIST_1] : Mv();
      const Mv mvQ0 = miQ.refIdx[REF_PIC_LIST_0] >= 0 ? miQ.mv[REF_PIC_LIST_0] : Mv();
      const Mv mvQ1 = miQ.refIdx[REF_PIC_LIST_1] >= 0 ? miQ.mv[REF_PIC_LIST_1] : Mv();

      if (refP0 != refP1)
      {
        d = refP0 == refQ0 ? motionBreak(mvP0, mvQ0) || motionBreak(mvP1, mvQ1)
                           : motionBreak(mvP0, mvQ1) || motionBreak(mvP1, mvQ0);
      }
      else
      {
        d = d && (motionBreak(mvP0, mvQ0) || motionBreak(mvP1, mvQ1));
        d = d && (motionBreak(mvP0, mvQ1) || motionBreak(mvP1, mvQ0));
      }
    }
  }
  else if (refP0 == refQ0)
  {
    d = motionBreak(miP.mv[REF_PIC_LIST_0], miQ.mv[REF_PIC_LIST_0]);
  }

  return tmpBs.setBoundaryStrength(COMPONENT_Y, d ? 1 : 0);
}

int DeblockingFilter::deriveLADFShift(const Pel *src, const int stride, const DeblockEdgeDir edgeDir, const SPS *sps)
{
  uint32_t lumaLevel = 0;

  if (edgeDir == EDGE_VER)
  {
    lumaLevel = (src[0] + src[3*stride] + src[-1] + src[3*stride - 1]) >> 2;
  }
  else // (edgeDir == EDGE_HOR)
  {
    lumaLevel = (src[0] + src[3] + src[-stride] + src[-stride + 3]) >> 2;
  }

  int shift = sps->getLadfQpOffset(0);

  for (int k = 1; k < sps->getLadfNumIntervals(); k++)
  {
    const int th = sps->getLadfIntervalLowerBound(k);
    if ( lumaLevel > th )
    {
      shift = sps->getLadfQpOffset(k);
    }
    else
    {
      break;
    }
  }

  return shift;
}

void DeblockingFilter::xEdgeFilterLuma(const CodingUnit &cu, const DeblockEdgeDir edgeDir, const int edgeIdx)
{
  const CompArea&  lumaArea = cu.block(COMPONENT_Y);
  const PreCalcValues& pcv = *cu.cs->pcv;

  PelBuf        picYuvRec = m_enc ? m_encPicYuvBuffer.getBuf( lumaArea ) : cu.cs->getRecoBuf( lumaArea );
  Pel *          src                            = picYuvRec.buf;
  const int      stride                         = picYuvRec.stride;
  Pel *          tmpSrc                         = src;
  const PPS     *pps                            = cu.cs->pps;
  const SPS     *sps                            = cu.cs->sps;
  const Slice   &slice    = *(cu.slice);
  const bool     spsPaletteEnabledFlag          = sps->getPLTMode();
  const int      bitDepthLuma                   = sps->getBitDepth(CHANNEL_TYPE_LUMA);
  const ClpRng& clpRng( cu.cs->slice->clpRng(COMPONENT_Y) );

  int      qp       = 0;
  unsigned numParts = (((edgeDir == EDGE_VER) ? lumaArea.height / pcv.minCUHeight : lumaArea.width / pcv.minCUWidth));
  int          pelsInPart   = pcv.minCUWidth;
  unsigned     bsAbsIdx = 0, bs = 0;
  int          offset, srcStep;

  bool  partPNoFilter   = false;
  bool  partQNoFilter   = false;
  int   betaOffsetDiv2  = slice.getDeblockingFilterBetaOffsetDiv2();
  int   tcOffsetDiv2    = slice.getDeblockingFilterTcOffsetDiv2();
  int   xoffset, yoffset;

  Position pos;

  if (edgeDir == EDGE_VER)
  {
    xoffset   = 0;
    yoffset   = pelsInPart;
    offset    = 1;
    srcStep   = stride;
    tmpSrc += edgeIdx * pelsInPart;
    pos = Position{ lumaArea.x + edgeIdx * pelsInPart, lumaArea.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset   = pelsInPart;
    yoffset   = 0;
    offset    = stride;
    srcStep   = 1;
    tmpSrc += edgeIdx * pelsInPart * stride;
    pos = Position{ lumaArea.x - xoffset, lumaArea.y + edgeIdx * pelsInPart };
  }

  const int bitdepthScale = 1 << (bitDepthLuma - 8);

  // dec pos since within the loop we first calc the pos
  for (int idx = 0; idx < numParts; idx++)
  {
    pos.x += xoffset;
    pos.y += yoffset;

    // Deblock luma boundaries on 4x4 grid only
    if (edgeDir == EDGE_HOR && (pos.y % 4) != 0)
    {
      continue;
    }
    if (edgeDir == EDGE_VER && (pos.x % 4) != 0)
    {
      continue;
    }
    bsAbsIdx = getRasterIdx(pos, pcv);
    bs       = m_edgeStrengths[edgeDir][bsAbsIdx].getBoundaryStrength(COMPONENT_Y);

    if (bs)
    {
      const CodingUnit& cuQ =  cu;
      const CodingUnit& cuP = *cu.cs->getCU(pos.offset(xoffset - pelsInPart, yoffset - pelsInPart), cu.chType);
      // Derive neighboring PU index
      if (edgeDir == EDGE_VER)
      {
        if (!isNeighbourAvailable(cu, cuP, *pps))
        {
          m_edgeStrengths[edgeDir][bsAbsIdx] = EdgeStrengths();
          continue;
        }
      }
      else   // (dir == EDGE_HOR)
      {
        if (!isNeighbourAvailable(cu, cuP, *pps))
        {
          m_edgeStrengths[edgeDir][bsAbsIdx] = EdgeStrengths();
          continue;
        }
      }

      qp = (cuP.qp + cuQ.qp + 1) >> 1;

      if (sps->getLadfEnabled())
      {
        const int shift = deriveLADFShift(tmpSrc + srcStep * (idx * pelsInPart), stride, edgeDir, sps);
        qp += shift;
      }

      bool sidePisLarge   = false;
      bool sideQisLarge   = false;

      auto maxFilterLen = m_maxFilterLen[CHANNEL_TYPE_LUMA][(pos.x - m_ctuXLumaSamples) >> LOG_GRID_SIZE]
                                        [(pos.y - m_ctuYLumaSamples) >> LOG_GRID_SIZE];

      if (maxFilterLen.p > FilterLen::_3)
      {
        sidePisLarge = true;
        if (maxFilterLen.p > FilterLen::_5)
        {
          // restrict filter length if sub-blocks are used (e.g affine or ATMVP)
          if (cuP.affine)
          {
            maxFilterLen.p = std::min(maxFilterLen.p, FilterLen::_5);
          }
        }
      }
      if (maxFilterLen.q > FilterLen::_3)
      {
        sideQisLarge = true;
      }

      if (edgeDir == EDGE_HOR && pos.y % slice.getSPS()->getCTUSize() == 0)
      {
        sidePisLarge = false;
      }
      const int indexTC =
        Clip3(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, int(qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + 2 * tcOffsetDiv2));
      const int indexB = Clip3(0, MAX_QP, qp + 2 * betaOffsetDiv2);

      const int tc   = bitDepthLuma < 10 ? ((sm_tcTable[indexTC] + (1 << (9 - bitDepthLuma))) >> (10 - bitDepthLuma))
                                         : ((sm_tcTable[indexTC]) << (bitDepthLuma - 10));
      const int beta          = sm_betaTable[indexB] * bitdepthScale;
      const int sideThreshold = (beta + (beta >> 1)) >> 3;
      const int thrCut        = tc * 10;

      const unsigned blocksInPart = pelsInPart / 4 ? pelsInPart / 4 : 1;

      for (int blkIdx = 0; blkIdx < blocksInPart; blkIdx++)
      {
        const int dp0  = xCalcDP(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 0), offset);
        const int dq0  = xCalcDQ(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 0), offset);
        const int dp3  = xCalcDP(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 3), offset);
        const int dq3  = xCalcDQ(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 3), offset);
        int dp0L = dp0;
        int dq0L = dq0;
        int dp3L = dp3;
        int dq3L = dq3;

        if (sidePisLarge)
        {
          dp0L = (dp0L + xCalcDP(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 0) - 3 * offset, offset) + 1) >> 1;
          dp3L = (dp3L + xCalcDP(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 3) - 3 * offset, offset) + 1) >> 1;
        }
        if (sideQisLarge)
        {
          dq0L = (dq0L + xCalcDQ(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 0) + 3 * offset, offset) + 1) >> 1;
          dq3L = (dq3L + xCalcDQ(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 3) + 3 * offset, offset) + 1) >> 1;
        }

        bool useLongtapFilter = false;
        if (sidePisLarge || sideQisLarge)
        {
          int d0L = dp0L + dq0L;
          int d3L = dp3L + dq3L;

          int dpL = dp0L + dp3L;
          int dqL = dq0L + dq3L;

          int dL = d0L + d3L;

          partPNoFilter = partQNoFilter = false;
          if (spsPaletteEnabledFlag)
          {
            // check if each of PUs is palette coded
            partPNoFilter = partPNoFilter || CU::isPLT(cuP);
            partQNoFilter = partQNoFilter || CU::isPLT(cuQ);
          }

          if (dL < beta)
          {
            const bool filterP = (dpL < sideThreshold);
            const bool filterQ = (dqL < sideThreshold);

            Pel *src0 = tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 0);
            Pel *src3 = tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 3);

            // adjust decision so that it is not read beyond p5 is maxFilterLenP is 5 and q5 if maxFilterLenQ is 5
            const bool swL =
              xUseStrongFiltering(src0, offset, 2 * d0L, beta, tc, sidePisLarge, sideQisLarge, maxFilterLen)
              && xUseStrongFiltering(src3, offset, 2 * d3L, beta, tc, sidePisLarge, sideQisLarge, maxFilterLen);
            if (swL)
            {
              useLongtapFilter = true;
              for (int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++)
              {
                xPelFilterLuma(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + i), offset, tc, swL, partPNoFilter,
                               partQNoFilter, thrCut, filterP, filterQ, clpRng, sidePisLarge, sideQisLarge,
                               maxFilterLen);
              }
            }

          }
        }
        if (!useLongtapFilter)
        {
          const int d0 = dp0 + dq0;
          const int d3 = dp3 + dq3;

          const int dp = dp0 + dp3;
          const int dq = dq0 + dq3;
          const int d  = d0 + d3;

          partPNoFilter = partQNoFilter = false;
          if (spsPaletteEnabledFlag)
          {
            // check if each of PUs is palette coded
            partPNoFilter = partPNoFilter || CU::isPLT(cuP);
            partQNoFilter = partQNoFilter || CU::isPLT(cuQ);
          }

          if (d < beta)
          {
            bool bFilterP = false;
            bool bFilterQ = false;
            if (maxFilterLen.p > FilterLen::_1 && maxFilterLen.q > FilterLen::_1)
            {
              bFilterP = (dp < sideThreshold);
              bFilterQ = (dq < sideThreshold);
            }
            bool sw = false;
            if (maxFilterLen.p > FilterLen::_2 && maxFilterLen.q > FilterLen::_2)
            {
              sw = xUseStrongFiltering(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 0), offset, 2 * d0, beta, tc)
                   && xUseStrongFiltering(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + 3), offset, 2 * d3, beta,
                                          tc);
            }
            for (int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++)
            {
              xPelFilterLuma(tmpSrc + srcStep * (idx * pelsInPart + blkIdx * 4 + i), offset, tc, sw, partPNoFilter,
                             partQNoFilter, thrCut, bFilterP, bFilterQ, clpRng);
            }
          }
        }
      }
    }
  }
}

void DeblockingFilter::xEdgeFilterChroma(const CodingUnit &cu, const DeblockEdgeDir edgeDir, const int edgeIdx)
{
  const Position lumaPos   = cu.Y().valid() ? cu.Y().pos() : recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() );
  const Size     lumaSize  = cu.Y().valid() ? cu.Y().size() : recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() );

  const PreCalcValues& pcv = *cu.cs->pcv;
  unsigned  rasterIdx      = getRasterIdx( lumaPos, pcv );
  PelBuf     picYuvRecCb = m_enc ? m_encPicYuvBuffer.getBuf(cu.block(COMPONENT_Cb)) : cu.cs->getRecoBuf(cu.block(COMPONENT_Cb));
  PelBuf     picYuvRecCr = m_enc ? m_encPicYuvBuffer.getBuf(cu.block(COMPONENT_Cr)) : cu.cs->getRecoBuf(cu.block(COMPONENT_Cr));
  Pel *      srcCb       = picYuvRecCb.buf;
  Pel *      srcCr       = picYuvRecCr.buf;
  const int          stride              = picYuvRecCb.stride;
  const SPS         &sps                 = *cu.cs->sps;
  const Slice  &slice      = *cu.slice;
  const ChromaFormat nChromaFormat   = sps.getChromaFormatIdc();

  const unsigned pelsInPartChromaH = pcv.minCUWidth >> ::getComponentScaleX(COMPONENT_Cb, nChromaFormat);
  const unsigned pelsInPartChromaV = pcv.minCUHeight >> ::getComponentScaleY(COMPONENT_Cb, nChromaFormat);

  int      offset, srcStep;
  unsigned loopLength;

  bool      partPNoFilter     = false;
  bool      partQNoFilter     = false;
  const int tcOffsetDiv2[2]   = { slice.getDeblockingFilterCbTcOffsetDiv2(), slice.getDeblockingFilterCrTcOffsetDiv2() };
  const int betaOffsetDiv2[2] = { slice.getDeblockingFilterCbBetaOffsetDiv2(), slice.getDeblockingFilterCrBetaOffsetDiv2() };

  // Vertical Position
  unsigned edgeNumInCtuVert = rasterIdx % pcv.partsInCtuWidth + edgeIdx;
  unsigned edgeNumInCtuHor  = rasterIdx / pcv.partsInCtuWidth + edgeIdx;

  if ((pelsInPartChromaH < DEBLOCK_SMALLEST_BLOCK) && (pelsInPartChromaV < DEBLOCK_SMALLEST_BLOCK)
      && (((edgeNumInCtuVert % (DEBLOCK_SMALLEST_BLOCK / pelsInPartChromaH)) && (edgeDir == EDGE_VER))
          || ((edgeNumInCtuHor % (DEBLOCK_SMALLEST_BLOCK / pelsInPartChromaV)) && (edgeDir == EDGE_HOR))))
  {
    return;
  }

  unsigned numParts    = (edgeDir == EDGE_VER) ? lumaSize.height / pcv.minCUHeight : lumaSize.width / pcv.minCUWidth;
  int      numPelsLuma = pcv.minCUWidth;

  Pel *tmpSrcCb = srcCb;
  Pel *tmpSrcCr = srcCr;

  int xoffset, yoffset;
  Position pos( lumaPos.x, lumaPos.y );

  if( edgeDir == EDGE_VER )
  {
    xoffset      = 0;
    yoffset      = numPelsLuma;
    offset       = 1;
    srcStep      = stride;
    tmpSrcCb += edgeIdx * pelsInPartChromaH;
    tmpSrcCr += edgeIdx * pelsInPartChromaH;
    loopLength = pelsInPartChromaV;
    pos        = Position{ lumaPos.x + edgeIdx * numPelsLuma, lumaPos.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset      = numPelsLuma;
    yoffset      = 0;
    offset       = stride;
    srcStep      = 1;
    tmpSrcCb += edgeIdx * stride * pelsInPartChromaV;
    tmpSrcCr += edgeIdx * stride * pelsInPartChromaV;
    loopLength = pelsInPartChromaH;
    pos        = Position{ lumaPos.x - xoffset, lumaPos.y + edgeIdx * numPelsLuma };
  }

  const int bitdepthScale = 1 << (sps.getBitDepth(CHANNEL_TYPE_CHROMA) - 8);

  for (int idx = 0; idx < numParts; idx++)
  {
    pos.x += xoffset;
    pos.y += yoffset;

    unsigned bsAbsIdx = getRasterIdx(pos, pcv);
    EdgeStrengths tmpBs    = m_edgeStrengths[edgeDir][bsAbsIdx];

    unsigned bS[2];
    bS[0] = tmpBs.getBoundaryStrength(COMPONENT_Cb);
    bS[1] = tmpBs.getBoundaryStrength(COMPONENT_Cr);

    if (bS[0] > 0 || bS[1] > 0)
    {
      const CodingUnit& cuQ =  cu;

      CodingUnit &cuP1 = *cu.cs->getCU(recalcPosition(cu.chromaFormat, CHANNEL_TYPE_LUMA, cu.chType,
                                                      pos.offset(xoffset - numPelsLuma, yoffset - numPelsLuma)),
                                       cu.chType);
      CodingUnit &cuP  = *cu.cs->getCU(recalcPosition(cu.chromaFormat, CHANNEL_TYPE_LUMA,
                                                     (cuP1.isSepTree() ? CHANNEL_TYPE_CHROMA : cu.chType),
                                                     pos.offset(xoffset - numPelsLuma, yoffset - numPelsLuma)),
                                      (cuP1.isSepTree() ? CHANNEL_TYPE_CHROMA : cu.chType));

      CHECKD(!isNeighbourAvailable(cu, cuP, *cu.cs->pps), "Neighbour not available");

      partPNoFilter = partQNoFilter = false;
      if ( sps.getPLTMode())
      {
        // check if each of PUs is palette coded
        partPNoFilter = partPNoFilter || CU::isPLT(cuP);
        partQNoFilter = partQNoFilter || CU::isPLT(cuQ);
      }

      const auto &maxFilterLen = m_maxFilterLen[CHANNEL_TYPE_CHROMA][(pos.x - m_ctuXLumaSamples) >> LOG_GRID_SIZE]
                                               [(pos.y - m_ctuYLumaSamples) >> LOG_GRID_SIZE];

      const bool largeBoundary = maxFilterLen.p >= FilterLen::_3 && maxFilterLen.q >= FilterLen::_3;

      bool isChromaHorCTBBoundary = false;

      if (edgeDir == EDGE_HOR && pos.y % cuP.slice->getSPS()->getCTUSize() == 0)
      {
        isChromaHorCTBBoundary = true;
      }

      for( int chromaIdx = 0; chromaIdx < 2; chromaIdx++ )
      {
        if ((bS[chromaIdx] == 2) || (largeBoundary && (bS[chromaIdx] == 1)))
        {
          const ClpRng &clpRng(cu.cs->slice->clpRng(ComponentID(chromaIdx + 1)));
          Pel *         tmpSrcChroma = (chromaIdx == 0) ? tmpSrcCb : tmpSrcCr;

          const TransformUnit &tuQ = *cuQ.cs->getTU(
            recalcPosition(cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA, pos), CHANNEL_TYPE_CHROMA);
          const TransformUnit &tuP =
            *cuP.cs->getTU(recalcPosition(cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA,
                                          (edgeDir == EDGE_VER) ? pos.offset(-1, 0) : pos.offset(0, -1)),
                           CHANNEL_TYPE_CHROMA);

          const QpParam qpP(tuP, ComponentID(chromaIdx + 1), -MAX_INT, false);
          const QpParam qpQ(tuQ, ComponentID(chromaIdx + 1), -MAX_INT, false);

          const int qpBdOffset = tuP.cs->sps->getQpBDOffset(toChannelType(ComponentID(chromaIdx + 1)));
          const int qp         = ((qpP.Qp(0) + qpQ.Qp(0) + 1) >> 1) - qpBdOffset;

          const int indexTC =
            Clip3<int>(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET,
                       qp + DEFAULT_INTRA_TC_OFFSET * (bS[chromaIdx] - 1) + 2 * tcOffsetDiv2[chromaIdx]);
          const int bitDepthChroma = sps.getBitDepth(CHANNEL_TYPE_CHROMA);
          const int tc             = bitDepthChroma < 10
                                       ? ((sm_tcTable[indexTC] + (1 << (9 - bitDepthChroma))) >> (10 - bitDepthChroma))
                                       : ((sm_tcTable[indexTC]) << (bitDepthChroma - 10));
          bool useLongFilter = false;
          if (largeBoundary)
          {
            const int indexB = Clip3<int>(0, MAX_QP, qp + 2 * betaOffsetDiv2[chromaIdx]);
            const int beta   = sm_betaTable[indexB] * bitdepthScale;

            const int dp0 = xCalcDP(tmpSrcChroma + srcStep * (idx * loopLength + 0), offset, isChromaHorCTBBoundary);
            const int dq0 = xCalcDQ(tmpSrcChroma + srcStep * (idx * loopLength + 0), offset);

            const int subSamplingShift = (edgeDir == EDGE_VER) ? m_shiftVer : m_shiftHor;

            const int dp3 =
              (subSamplingShift == 1)
                ? xCalcDP(tmpSrcChroma + srcStep * (idx * loopLength + 1), offset, isChromaHorCTBBoundary)
                : xCalcDP(tmpSrcChroma + srcStep * (idx * loopLength + 3), offset, isChromaHorCTBBoundary);
            const int dq3 = (subSamplingShift == 1) ? xCalcDQ(tmpSrcChroma + srcStep * (idx * loopLength + 1), offset)
                                                    : xCalcDQ(tmpSrcChroma + srcStep * (idx * loopLength + 3), offset);

            const int d0 = dp0 + dq0;
            const int d3 = dp3 + dq3;
            const int d  = d0 + d3;

            if (d < beta)
            {
              useLongFilter = true;
              const bool sw =
                xUseStrongFiltering(tmpSrcChroma + srcStep * (idx * loopLength + 0), offset, 2 * d0, beta, tc, false,
                                    false, DEFAULT_FL2, isChromaHorCTBBoundary)
                && xUseStrongFiltering(tmpSrcChroma + srcStep * (idx * loopLength + ((subSamplingShift == 1) ? 1 : 3)),
                                       offset, 2 * d3, beta, tc, false, false, DEFAULT_FL2, isChromaHorCTBBoundary);

              for (unsigned step = 0; step < loopLength; step++)
              {
                xPelFilterChroma(tmpSrcChroma + srcStep * (step + idx * loopLength), offset, tc, sw, partPNoFilter,
                                 partQNoFilter, clpRng, largeBoundary, isChromaHorCTBBoundary);
              }
            }
          }
          if (!useLongFilter)
          {
            for (unsigned step = 0; step < loopLength; step++)
            {
              xPelFilterChroma(tmpSrcChroma + srcStep * (step + idx * loopLength), offset, tc, false, partPNoFilter,
                               partQNoFilter, clpRng, largeBoundary, isChromaHorCTBBoundary);
            }
          }
        }
      }
    }
  }
}

void DeblockingFilter::xFilteringPandQ(Pel *src, int offset, const FilterLenPair filterLen, int tc)
{
  CHECK(filterLen.p <= FilterLen::_3 && filterLen.q <= FilterLen::_3, "Short filtering in long filtering function");
  Pel *srcP = src - offset;
  Pel* srcQ = src;

  using A = static_vector<char, 8>;

  static const EnumArray<A, FilterLen> dbCoeffsX = {
    A{}, A{}, A{ 53, 32, 11 }, A{ 58, 45, 32, 19, 6 }, A{ 59, 50, 41, 32, 23, 14, 5 },
  };

  const auto &dbCoeffsP = dbCoeffsX[filterLen.p];
  const auto &dbCoeffsQ = dbCoeffsX[filterLen.q];

  const auto nP = (int) dbCoeffsP.size();
  const auto nQ = (int) dbCoeffsQ.size();

  const int refP = (srcP[-(nP * offset - offset)] + srcP[-(nP * offset)] + 1) >> 1;
  const int refQ = (srcQ[nQ * offset - offset] + srcQ[nQ * offset] + 1) >> 1;

  int refMiddle = 0;

  if (filterLen.p == filterLen.q)
  {
    if (filterLen.p == FilterLen::_5)
    {
      refMiddle = (2 * (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset]) + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + 8) >> 4;
    }
    else
    {
      refMiddle = (2 * (srcP[0] + srcQ[0]) + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + +srcP[-6 * offset] + srcQ[6 * offset] + 8) >> 4;
    }
  }
  else
  {
    Pel* srcPt = srcP;
    Pel* srcQt = srcQ;
    int offsetP = -offset;
    int offsetQ = offset;

    FilterLen newNumberQSide = filterLen.q;
    FilterLen newNumberPSide = filterLen.p;
    if (newNumberQSide > newNumberPSide)
    {
      std::swap(srcPt, srcQt);
      std::swap(offsetP, offsetQ);
      std::swap(newNumberPSide, newNumberQSide);
    }

    if (newNumberPSide == FilterLen::_7 && newNumberQSide == FilterLen::_5)
    {
      refMiddle = (2 * (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset]) + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + 8) >> 4;
    }
    else if (newNumberPSide == FilterLen::_7)
    {
      CHECKD(newNumberQSide != FilterLen::_3, "bad filter length");
      refMiddle = (2 * (srcPt[0] + srcQt[0]) + srcQt[0] + 2 * (srcQt[offsetQ] + srcQt[2 * offsetQ]) + srcPt[offsetP] + srcQt[offsetQ] + srcPt[2 * offsetP] + srcPt[3 * offsetP] + srcPt[4 * offsetP] + srcPt[5 * offsetP] + srcPt[6 * offsetP] + 8) >> 4;
    }
    else
    {
      CHECKD(newNumberPSide != FilterLen::_5 || newNumberQSide != FilterLen::_3, "bad filter length");
      refMiddle = (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + 4) >> 3;
    }
  }

  static const EnumArray<A, FilterLen> tcX = { A{}, A{}, A{ 6, 4, 2 }, A{ 6, 5, 4, 3, 2 }, A{ 6, 5, 4, 3, 2, 1, 1 } };

  const auto &tcP = tcX[filterLen.p];
  const auto &tcQ = tcX[filterLen.q];

  for (int pos = 0; pos < tcP.size(); pos++)
  {
    int src    = srcP[-offset * pos];
    int cvalue = tc * tcP[pos] >> 1;
    srcP[-offset * pos] =
      Clip3(src - cvalue, src + cvalue, ((refMiddle * dbCoeffsP[pos] + refP * (64 - dbCoeffsP[pos]) + 32) >> 6));
  }

  for (int pos = 0; pos < tcQ.size(); pos++)
  {
    int src    = srcQ[offset * pos];
    int cvalue = tc * tcQ[pos] >> 1;
    srcQ[offset * pos] =
      Clip3(src - cvalue, src + cvalue, ((refMiddle * dbCoeffsQ[pos] + refQ * (64 - dbCoeffsQ[pos]) + 32) >> 6));
  }
}

void DeblockingFilter::xPelFilterLuma(Pel *src, const int offset, const int tc, const bool sw, const bool partPNoFilter,
                                      const bool partQNoFilter, const int thrCut, const bool bFilterSecondP,
                                      const bool bFilterSecondQ, const ClpRng &clpRng, bool sidePisLarge,
                                      bool sideQisLarge, FilterLenPair maxFilterLen)
{
  int delta;

  const Pel m4 = src[0];
  const Pel m3 = src[-offset];
  const Pel m5 = src[offset];
  const Pel m2 = src[-offset * 2];
  const Pel m6 = src[offset * 2];
  const Pel m1 = src[-offset * 3];
  const Pel m7 = src[offset * 3];
  const Pel m0 = src[-offset * 4];

  const Pel mP1 = src[-offset * 5];
  const Pel mP2 = src[-offset * 6];
  const Pel mP3 = src[-offset * 7];
  const Pel m8  = src[offset * 4];
  const Pel m9  = src[offset * 5];
  const Pel m10 = src[offset * 6];

  if (sw)
  {
    if (sidePisLarge || sideQisLarge)
    {
      if (!sidePisLarge)
      {
        maxFilterLen.p = FilterLen::_3;
      }
      if (!sideQisLarge)
      {
        maxFilterLen.q = FilterLen::_3;
      }
      xFilteringPandQ(src, offset, maxFilterLen, tc);
    }
    else
    {
      src[-offset]     = Clip3(m3 - 3 * tc, m3 + 3 * tc, (m1 + 2 * m2 + 2 * m3 + 2 * m4 + m5 + 4) >> 3);
      src[0]           = Clip3(m4 - 3 * tc, m4 + 3 * tc, (m2 + 2 * m3 + 2 * m4 + 2 * m5 + m6 + 4) >> 3);
      src[-offset * 2] = Clip3(m2 - 2 * tc, m2 + 2 * tc, (m1 + m2 + m3 + m4 + 2) >> 2);
      src[offset]      = Clip3(m5 - 2 * tc, m5 + 2 * tc, (m3 + m4 + m5 + m6 + 2) >> 2);
      src[-offset * 3] = Clip3(m1 - 1 * tc, m1 + 1 * tc, (2 * m0 + 3 * m1 + m2 + m3 + m4 + 4) >> 3);
      src[offset * 2]  = Clip3(m6 - 1 * tc, m6 + 1 * tc, (m3 + m4 + m5 + 3 * m6 + 2 * m7 + 4) >> 3);
    }
  }
  else
  {
    /* Weak filter */
    delta = ( 9 * ( m4 - m3 ) - 3 * ( m5 - m2 ) + 8 ) >> 4;

    if (abs(delta) < thrCut)
    {
      delta = Clip3( -tc, tc, delta );
      src[-offset] = ClipPel(m3 + delta, clpRng);
      src[0]       = ClipPel(m4 - delta, clpRng);

      const int tc2 = tc >> 1;
      if( bFilterSecondP )
      {
        const int delta1 = Clip3( -tc2, tc2, ( ( ( ( m1 + m3 + 1 ) >> 1 ) - m2 + delta ) >> 1 ) );
        src[-offset * 2] = ClipPel(m2 + delta1, clpRng);
      }
      if( bFilterSecondQ )
      {
        const int delta2 = Clip3( -tc2, tc2, ( ( ( ( m6 + m4 + 1 ) >> 1 ) - m5 - delta ) >> 1 ) );
        src[offset]      = ClipPel(m5 + delta2, clpRng);
      }
    }
  }

  if (partPNoFilter)
  {
    src[-offset]     = m3;
    src[-offset * 2] = m2;
    src[-offset * 3] = m1;
    if (sidePisLarge)
    {
      src[-offset * 4] = m0;
      src[-offset * 5] = mP1;
      src[-offset * 6] = mP2;
      src[-offset * 7] = mP3;
    }
  }

  if (partQNoFilter)
  {
    src[0]          = m4;
    src[offset]     = m5;
    src[offset * 2] = m6;
    if (sideQisLarge)
    {
      src[offset * 3] = m7;
      src[offset * 4] = m8;
      src[offset * 5] = m9;
      src[offset * 6] = m10;
    }
  }
}

inline void DeblockingFilter::xPelFilterChroma(Pel *src, const int offset, const int tc, const bool sw,
                                               const bool partPNoFilter, const bool partQNoFilter, const ClpRng &clpRng,
                                               const bool largeBoundary, const bool isChromaHorCTBBoundary) const
{
  int delta;

  const Pel m0 = src[-offset * 4];
  const Pel m1 = src[-offset * 3];
  const Pel m2 = src[-offset * 2];
  const Pel m3 = src[-offset];
  const Pel m4 = src[0];
  const Pel m5 = src[offset];
  const Pel m6 = src[offset * 2];
  const Pel m7 = src[offset * 3];

  if (sw)
  {
    if (isChromaHorCTBBoundary)
    {
      src[-offset * 1] = Clip3(m3 - tc, m3 + tc, ((3 * m2 + 2 * m3 + m4 + m5 + m6 + 4) >> 3));        // p0
      src[0]           = Clip3(m4 - tc, m4 + tc, ((2 * m2 + m3 + 2 * m4 + m5 + m6 + m7 + 4) >> 3));   // q0
      src[offset * 1]  = Clip3(m5 - tc, m5 + tc, ((m2 + m3 + m4 + 2 * m5 + m6 + 2 * m7 + 4) >> 3));   // q1
      src[offset * 2]  = Clip3(m6 - tc, m6 + tc, ((m3 + m4 + m5 + 2 * m6 + 3 * m7 + 4) >> 3));        // q2
    }
    else
    {
      src[-offset * 3] = Clip3(m1 - tc, m1 + tc, ((3 * m0 + 2 * m1 + m2 + m3 + m4 + 4) >> 3));         // p2
      src[-offset * 2] = Clip3(m2 - tc, m2 + tc, ((2 * m0 + m1 + 2 * m2 + m3 + m4 + m5 + 4) >> 3));    // p1
      src[-offset * 1] = Clip3(m3 - tc, m3 + tc, ((m0 + m1 + m2 + 2 * m3 + m4 + m5 + m6 + 4) >> 3));   // p0
      src[0]           = Clip3(m4 - tc, m4 + tc, ((m1 + m2 + m3 + 2 * m4 + m5 + m6 + m7 + 4) >> 3));   // q0
      src[offset * 1]  = Clip3(m5 - tc, m5 + tc, ((m2 + m3 + m4 + 2 * m5 + m6 + 2 * m7 + 4) >> 3));    // q1
      src[offset * 2]  = Clip3(m6 - tc, m6 + tc, ((m3 + m4 + m5 + 2 * m6 + 3 * m7 + 4) >> 3));         // q2
    }
  }
  else
  {
    delta           = Clip3(-tc, tc, ((4 * (m4 - m3) + m2 - m5 + 4) >> 3));
    src[-offset]    = ClipPel(m3 + delta, clpRng);
    src[0]          = ClipPel(m4 - delta, clpRng);
  }

  if (partPNoFilter)
  {
    if (largeBoundary)
    {
      src[-offset * 3] = m1;   // p2
      src[-offset * 2] = m2;   // p1
    }
    src[-offset] = m3;
  }
  if (partQNoFilter)
  {
    if (largeBoundary)
    {
      src[offset * 1] = m5;   // q1
      src[offset * 2] = m6;   // q2
    }
    src[0] = m4;
  }
}

inline bool DeblockingFilter::xUseStrongFiltering(Pel *src, const int offset, const int d, const int beta, const int tc,
                                                  bool sidePisLarge, bool sideQisLarge, FilterLenPair maxFilterLen,
                                                  bool isChromaHorCTBBoundary) const
{
  const Pel m4  = src[0];
  const Pel m3  = src[-offset];
  const Pel m7  = src[offset * 3];
  const Pel m0  = src[-offset * 4];
  const Pel m2  = src[-offset * 2];

  int sp3 = isChromaHorCTBBoundary ? abs(m2 - m3) : abs(m0 - m3);
  int sq3 = abs(m7 - m4);

  if (sidePisLarge || sideQisLarge)
  {
    Pel mP4;
    Pel m11;
    if (sidePisLarge)
    {
      if (maxFilterLen.p == FilterLen::_7)
      {
        const Pel mP5 = src[-offset * 5];
        const Pel mP6 = src[-offset * 6];
        const Pel mP7 = src[-offset * 7];

        mP4 = src[-offset * 8];
        sp3 = sp3 + abs(mP5 - mP6 - mP7 + mP4);
      }
      else
      {
        mP4 = src[-offset * 6];
      }
      sp3 = (sp3 + abs(m0 - mP4) + 1) >> 1;
    }
    if (sideQisLarge)
    {
      if (maxFilterLen.q == FilterLen::_7)
      {
        const Pel m8  = src[offset * 4];
        const Pel m9  = src[offset * 5];
        const Pel m10 = src[offset * 6];

        m11 = src[offset * 7];
        sq3 = sq3 + abs(m8 - m9 - m10 + m11);
      }
      else
      {
        m11 = src[offset * 5];
      }
      sq3 = (sq3 + abs(m11 - m7) + 1) >> 1;
    }
    return sp3 + sq3 < (beta * 3 >> 5) && d < (beta >> 4) && abs(m3 - m4) < ((tc * 5 + 1) >> 1);
  }
  else
  {
    return sp3 + sq3 < (beta >> 3) && d < (beta >> 2) && abs(m3 - m4) < ((tc * 5 + 1) >> 1);
  }
}

inline int DeblockingFilter::xCalcDP(Pel *src, const int offset, const bool isChromaHorCTBBoundary) const
{
  if (isChromaHorCTBBoundary)
  {
    return abs(src[-offset * 2] - 2 * src[-offset * 2] + src[-offset]);
  }
  else
  {
    return abs(src[-offset * 3] - 2 * src[-offset * 2] + src[-offset]);
  }
}

inline int DeblockingFilter::xCalcDQ(Pel *src, const int offset) const
{
  return abs(src[0] - 2 * src[offset] + src[offset * 2]);
}

//! \}
