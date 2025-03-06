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

 /** \file     SEIPackedRegionsInfoProcess.cpp
     \brief    Packed regions info SEI processing
 */

#include "SequenceParameterSet.h"
#include "Picture.h"
#include "SEIPackedRegionsInfoProcess.h"


void SEIPackedRegionsInfoProcess::init(SEIPackedRegionsInfo& sei, const SPS& sps, uint32_t picWidth, uint32_t picHeight)
{
  m_enabled = true;
  m_persistence = sei.m_persistenceFlag;
  m_priUnitSize = 1 << sei.m_log2UnitSize;
  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxPicWidth = sps.getMaxPicWidthInLumaSamples();
  m_maxPicHeight = sps.getMaxPicHeightInLumaSamples();
  m_targetPicWidth = 0;
  m_targetPicHeight = 0;
  if (sei.m_targetPicParamsPresentFlag)
  {
    m_targetPicWidth = sei.m_targetPicWidthMinus1 + 1;
    m_targetPicHeight = sei.m_targetPicHeightMinus1 + 1;
  }
  m_bitDepthY = sps.getBitDepth(ChannelType::LUMA);
  m_bitDepthC = sps.getBitDepth(ChannelType::CHROMA);
  m_chromaFormat = sps.getChromaFormatIdc();
  m_subWidthC = SPS::getWinUnitX(sps.getChromaFormatIdc());
  m_subHeightC = SPS::getWinUnitY(sps.getChromaFormatIdc());

  m_priNumRegions = sei.m_numRegionsMinus1 + 1;
  m_priRegionTopLeftX.resize(sei.m_numRegionsMinus1 + 1);
  m_priRegionTopLeftY.resize(sei.m_numRegionsMinus1 + 1);
  m_priRegionWidth.resize(sei.m_numRegionsMinus1 + 1);
  m_priRegionHeight.resize(sei.m_numRegionsMinus1 + 1);
  m_priResampleWidthNum.resize(sei.m_numRegionsMinus1 + 1);
  m_priResampleWidthDenom.resize(sei.m_numRegionsMinus1 + 1);
  m_priResampleHeightNum.resize(sei.m_numRegionsMinus1 + 1);
  m_priResampleHeightDenom.resize(sei.m_numRegionsMinus1 + 1);
  m_priTargetRegionWidth.resize(sei.m_numRegionsMinus1 + 1);
  m_priTargetRegionHeight.resize(sei.m_numRegionsMinus1 + 1);
  m_priRegionId.resize(sei.m_numRegionsMinus1 + 1);

  for (uint32_t i = 0; i <= sei.m_numRegionsMinus1; i++)
  {
    if (!sei.m_useMaxDimensionsFlag)
    {
      m_priRegionTopLeftX[i] = sei.m_regionTopLeftInUnitsX[i] * m_priUnitSize;
      m_priRegionTopLeftY[i] = sei.m_regionTopLeftInUnitsY[i] * m_priUnitSize;
      m_priRegionWidth[i] = (sei.m_regionWidthInUnitsMinus1[i] + 1) * m_priUnitSize;
      m_priRegionHeight[i] = (sei.m_regionHeightInUnitsMinus1[i] + 1) * m_priUnitSize;
    }
    else
    {
      m_priRegionTopLeftX[i] = (sei.m_regionTopLeftInUnitsX[i] * m_priUnitSize * m_picWidth + m_maxPicWidth/2) / m_maxPicWidth;
      m_priRegionTopLeftY[i] = (sei.m_regionTopLeftInUnitsY[i] * m_priUnitSize * m_picHeight + m_maxPicHeight/2) / m_maxPicHeight;
      m_priRegionWidth[i] = ((sei.m_regionWidthInUnitsMinus1[i] + 1) * m_priUnitSize * m_picWidth + m_maxPicWidth/2) / m_maxPicWidth;
      m_priRegionHeight[i] = ((sei.m_regionHeightInUnitsMinus1[i] + 1) * m_priUnitSize * m_picHeight + m_maxPicHeight/2) / m_maxPicHeight;
    }
    uint32_t resamplingRatioIdx = 0;
    if (sei.m_numResamplingRatiosMinus1 > 0)
    {
      resamplingRatioIdx = sei.m_resamplingRatioIdx[i];
    }
    m_priResampleWidthNum[i] = sei.m_resamplingWidthNumMinus1[resamplingRatioIdx] + 1;
    m_priResampleWidthDenom[i] = sei.m_resamplingWidthDenomMinus1[resamplingRatioIdx] + 1;
    m_priResampleHeightNum[i] = sei.m_resamplingHeightNumMinus1[resamplingRatioIdx] + 1;
    m_priResampleHeightDenom[i] = sei.m_resamplingHeightDenomMinus1[resamplingRatioIdx] + 1;
    m_priTargetRegionWidth[i] = ((uint32_t)(((double)m_priRegionWidth[i] * m_priResampleWidthNum[i]) / (m_priResampleWidthDenom[i] * m_subWidthC) + 0.5)) * m_subWidthC;
    m_priTargetRegionHeight[i] = ((uint32_t)(((double)m_priRegionHeight[i] * m_priResampleHeightNum[i]) / (m_priResampleHeightDenom[i] * m_subHeightC) + 0.5)) * m_subHeightC;
  }
  m_priTargetRegionTopLeftX = sei.m_targetRegionTopLeftX;
  m_priTargetRegionTopLeftY = sei.m_targetRegionTopLeftY;
  m_priRegionId = sei.m_regionId;
}

void SEIPackedRegionsInfoProcess::packRegions(PelUnitBuf& src, PelUnitBuf& dst, const SPS& sps)
{
  for (uint32_t i = 0; i < m_priNumRegions; i++)
  {
    int xScale = ((m_priTargetRegionWidth[i] << ScalingRatio::BITS) + (m_priRegionWidth[i] >> 1)) / m_priRegionWidth[i];
    int yScale = ((m_priTargetRegionHeight[i] << ScalingRatio::BITS) + (m_priRegionHeight[i] >> 1)) / m_priRegionHeight[i];
    ScalingRatio scalingRatio = { xScale, yScale };
    for (int comp = 0; comp < ::getNumberValidComponents(m_chromaFormat); comp++)
    {
      ComponentID compID = ComponentID(comp);
      const CPelBuf& beforeScale = src.get(compID);
      PelBuf& afterScale = dst.get(compID);
      int cx = isLuma(compID) ? 1 : m_subWidthC;
      int cy = isLuma(compID) ? 1 : m_subHeightC;
      CPelBuf beforeScaleSub = beforeScale.subBuf(m_priTargetRegionTopLeftX[i] / cx, m_priTargetRegionTopLeftY[i] / cy, m_priTargetRegionWidth[i] / cx, m_priTargetRegionHeight[i] / cy);
      PelBuf afterScaleSub = afterScale.subBuf(m_priRegionTopLeftX[i] / cx, m_priRegionTopLeftY[i] / cy, m_priRegionWidth[i] / cx, m_priRegionHeight[i] / cy);
      bool downsampling = (m_priTargetRegionWidth[i] > m_priRegionWidth[i]) || (m_priTargetRegionHeight[i] > m_priRegionHeight[i]);
      bool useLumaFilter = downsampling;
      Picture::sampleRateConv(
        scalingRatio, ::getComponentScaleX(compID, m_chromaFormat), ::getComponentScaleY(compID, m_chromaFormat),
        beforeScaleSub, 0, 0, afterScaleSub, 0, 0, isLuma(compID) ? m_bitDepthY : m_bitDepthC,
        downsampling || useLumaFilter ? true : isLuma(compID), downsampling, isLuma(compID) ? 1 : sps.getHorCollocatedChromaFlag(),
        isLuma(compID) ? 1 : sps.getVerCollocatedChromaFlag(), false, false);
    }
  }
}

void SEIPackedRegionsInfoProcess::reconstruct(const PelUnitBuf& src, PelUnitBuf& dst, const SPS& sps, const PPS& pps)
{
  Window win = pps.getConformanceWindow();
  int winLeftOffset = win.getWindowLeftOffset() * m_subWidthC;
  int winTopOffset = win.getWindowTopOffset() * m_subHeightC;

  for (int comp = 0; comp < ::getNumberValidComponents(m_chromaFormat); comp++)
  {
    ComponentID compID = ComponentID(comp);
    dst.get(compID).fill(1 << ((isLuma(compID) ? m_bitDepthY : m_bitDepthC) - 1));
  }

  uint32_t maxId = *std::max_element(m_priRegionId.begin(), m_priRegionId.end());
  for (uint32_t regionId = 0; regionId <= maxId; regionId++)
  {
    auto it = std::find(m_priRegionId.begin(), m_priRegionId.end(), regionId);
    if (it != m_priRegionId.end())
    {
      int i = (int)(it - m_priRegionId.begin());
      int xScale = ((m_priRegionWidth[i] << ScalingRatio::BITS) + (m_priTargetRegionWidth[i] >> 1)) / m_priTargetRegionWidth[i];
      int yScale = ((m_priRegionHeight[i] << ScalingRatio::BITS) + (m_priTargetRegionHeight[i] >> 1)) / m_priTargetRegionHeight[i];
      ScalingRatio scalingRatio = { xScale, yScale };
      for (int comp = 0; comp < ::getNumberValidComponents(m_chromaFormat); comp++)
      {
        ComponentID compID = ComponentID(comp);
        const CPelBuf& beforeScale = src.get(compID);
        PelBuf& afterScale = dst.get(compID);
        int cx = isLuma(compID) ? 1 : m_subWidthC;
        int cy = isLuma(compID) ? 1 : m_subHeightC;
        CPelBuf beforeScaleSub = beforeScale.subBuf((winLeftOffset + m_priRegionTopLeftX[i]) / cx, (winTopOffset + m_priRegionTopLeftY[i]) / cy, m_priRegionWidth[i] / cx, m_priRegionHeight[i] / cy);
        PelBuf afterScaleSub = afterScale.subBuf(m_priTargetRegionTopLeftX[i] / cx, m_priTargetRegionTopLeftY[i] / cy, m_priTargetRegionWidth[i] / cx, m_priTargetRegionHeight[i] / cy);
        bool downsampling = (m_priRegionWidth[i] > m_priTargetRegionWidth[i]) || (m_priRegionHeight[i] > m_priTargetRegionHeight[i]);
        bool useLumaFilter = downsampling;
        Picture::sampleRateConv(
          scalingRatio, ::getComponentScaleX(compID, m_chromaFormat), ::getComponentScaleY(compID, m_chromaFormat),
          beforeScaleSub, 0, 0, afterScaleSub, 0, 0, isLuma(compID) ? m_bitDepthY : m_bitDepthC,
          downsampling || useLumaFilter ? true : isLuma(compID), downsampling, isLuma(compID) ? 1 : sps.getHorCollocatedChromaFlag(),
          isLuma(compID) ? 1 : sps.getVerCollocatedChromaFlag(), false, false);
      }
    }
  }
}
