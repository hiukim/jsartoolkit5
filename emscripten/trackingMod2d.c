/*
 *  AR2/tracking2d.c
 *  ARToolKit5
 *
 *  This file is part of ARToolKit.
 *
 *  ARToolKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ARToolKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ARToolKit.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2006-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb
 *  Modified version by @misdake
 *
 */

#include <AR/ar.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <AR2/config.h>
#include <AR2/featureSet.h>
#include <AR2/template.h>
#include <AR2/searchPoint.h>
#include <AR2/tracking.h>

#include <emscripten.h>

#if AR2_CAPABLE_ADAPTIVE_TEMPLATE
int ar2Tracking2dSub ( AR2HandleT *handle, AR2SurfaceSetT *surfaceSet, AR2TemplateCandidateT *candidate,
                              ARUint8 *dataPtr, ARUint8 *mfImage, AR2TemplateT **templ,
                              AR2Template2T **templ2, AR2Tracking2DResultT *result );
#else
int ar2Tracking2dSub ( AR2HandleT *handle, AR2SurfaceSetT *surfaceSet, AR2TemplateCandidateT *candidate,
                              ARUint8 *dataPtr, ARUint8 *mfImage, AR2TemplateT **templ,
                              AR2Tracking2DResultT *result );
#endif

#if AR2_CAPABLE_ADAPTIVE_TEMPLATE
int ar2Tracking2dSub ( AR2HandleT *handle, AR2SurfaceSetT *surfaceSet, AR2TemplateCandidateT *candidate,
                              ARUint8 *dataPtr, ARUint8 *mfImage, AR2TemplateT **templ,
                              AR2Template2T **templ2, AR2Tracking2DResultT *result )
#else
int ar2Tracking2dSub ( AR2HandleT *handle, AR2SurfaceSetT *surfaceSet, AR2TemplateCandidateT *candidate,
                              ARUint8 *dataPtr, ARUint8 *mfImage, AR2TemplateT **templ,
                              AR2Tracking2DResultT *result )
#endif
{
#if AR2_CAPABLE_ADAPTIVE_TEMPLATE
    AR2Template2T        *templ2;
#endif
    int                   snum, level, fnum;
    int                   search[3][2];
    int                   bx, by;

    snum  = candidate->snum;
    level = candidate->level;
    fnum  = candidate->num;

    EM_ASM_({
      var a = arguments;
      artoolkit.kimDebugMatching.tracking2dSub.push({
        snum: a[0],
        leve: a[1],
        fnum: a[2],
        templateCompute: ([]),
        matchingCompute: ([]),
        matchingComputeSum: ([]),
        matchingCandidates: ([]),
        bestMatchingCompute: ([]),
        bestMatched: ([]),
      });
    }, snum, level, fnum);

    if( *templ == NULL )  *templ = ar2GenTemplate( handle->templateSize1, handle->templateSize2 );
#if AR2_CAPABLE_ADAPTIVE_TEMPLATE
    if( *templ2 == NULL ) *templ2 = ar2GenTemplate2( handle->templateSize1, handle->templateSize2 );
#endif

#if AR2_CAPABLE_ADAPTIVE_TEMPLATE
    if( handle->blurMethod == AR2_CONSTANT_BLUR ) {
        if( ar2SetTemplateSub( handle->cparamLT,
                               (const float (*)[4])handle->wtrans1[snum],
                               surfaceSet->surface[snum].imageSet,
                             &(surfaceSet->surface[snum].featureSet->list[level]),
                               fnum,
                               handle->blurLevel,
                              *templ ) < 0 ) return -1;

        if( (*templ)->vlen * (*templ)->vlen
              < ((*templ)->xts1+(*templ)->xts2+1) * ((*templ)->yts1+(*templ)->yts2+1)
               * AR2_DEFAULT_TRACKING_SD_THRESH * AR2_DEFAULT_TRACKING_SD_THRESH ) {
            return -1;
        }
    }
    else {
        if( ar2SetTemplate2Sub( handle->cparamLT,
                                (const float (*)[4])handle->wtrans1[snum],
                                surfaceSet->surface[snum].imageSet,
                              &(surfaceSet->surface[snum].featureSet->list[level]),
                                fnum,
                                handle->blurLevel,
                               *templ2 ) < 0 ) return -1;

        if( (*templ2)->vlen[1] * (*templ2)->vlen[1]
              < ((*templ2)->xts1+(*templ2)->xts2+1) * ((*templ2)->yts1+(*templ2)->yts2+1)
               * AR2_DEFAULT_TRACKING_SD_THRESH * AR2_DEFAULT_TRACKING_SD_THRESH ) {
            return -1;
        }
    }
#else
    if( ar2SetTemplateSub( handle->cparamLT,
                           (const float (*)[4])handle->wtrans1[snum],
                           surfaceSet->surface[snum].imageSet,
                         &(surfaceSet->surface[snum].featureSet->list[level]),
                           fnum,
                          *templ ) < 0 ) return -1;

    if( (*templ)->vlen * (*templ)->vlen
          < ((*templ)->xts1 + (*templ)->xts2 + 1) * ((*templ)->yts1 + (*templ)->yts2 + 1)
           * AR2_DEFAULT_TRACKING_SD_THRESH * AR2_DEFAULT_TRACKING_SD_THRESH ) {
        return -1;
    }

    EM_ASM_({
      var a = arguments;
      var s = artoolkit.kimDebugMatching.tracking2dSub[artoolkit.kimDebugMatching.tracking2dSub.length-1];
      s.vlen = a[0];
      s.sum = a[1];
      s.validNum = a[2];
      s.yts1 = a[3];
      s.yts2 = a[4];
      s.xts1 = a[5];
      s.xts2 = a[6];
      s.template = ([]);
    }, (*templ)->vlen, (*templ)->sum, (*templ)->validNum, (*templ)->yts1, (*templ)->yts2, (*templ)->xts1, (*templ)->xts2);

    int iii = 0;
    for(int j = -((*templ)->yts1); j <= (*templ)->yts2; j++) {
      for(int i = -((*templ)->xts1); i <= (*templ)->xts2; i++) {
        EM_ASM_({
          var a = arguments;
          var s = artoolkit.kimDebugMatching.tracking2dSub[artoolkit.kimDebugMatching.tracking2dSub.length-1];
          s.template.push(a[0]);
        }, (*templ)->img1[iii]);
        iii += 1;
      }
    }

#endif

    // Get the screen coordinates for up to three previous positions of this feature into search[][].
    if( surfaceSet->contNum == 1 ) {
        ar2GetSearchPoint( handle->cparamLT,
                           (const float (*)[4])handle->wtrans1[snum], NULL, NULL,
                         &(surfaceSet->surface[snum].featureSet->list[level].coord[fnum]),
                           search );
    }
    else if( surfaceSet->contNum == 2 ) {
        ar2GetSearchPoint( handle->cparamLT,
                           (const float (*)[4])handle->wtrans1[snum],
                           (const float (*)[4])handle->wtrans2[snum], NULL,
                         &(surfaceSet->surface[snum].featureSet->list[level].coord[fnum]),
                           search );
    }
    else {
        ar2GetSearchPoint( handle->cparamLT,
                           (const float (*)[4])handle->wtrans1[snum],
                           (const float (*)[4])handle->wtrans2[snum],
                           (const float (*)[4])handle->wtrans3[snum],
                         &(surfaceSet->surface[snum].featureSet->list[level].coord[fnum]),
                           search );
    }

    EM_ASM_({
      var a = arguments;
      var s = artoolkit.kimDebugMatching.tracking2dSub[artoolkit.kimDebugMatching.tracking2dSub.length-1];
      s.search = ({
        s1: ([a[0], a[1]]),
        s2: ([a[2], a[3]]),
        s3: ([a[4], a[5]]),
        m1: ([a[6], a[7]])
      });
    }, search[0][0], search[0][1], search[1][0], search[1][1], search[2][0], search[2][1], surfaceSet->surface[snum].featureSet->list[level].coord[fnum].mx, surfaceSet->surface[snum].featureSet->list[level].coord[fnum].my);

#if AR2_CAPABLE_ADAPTIVE_TEMPLATE
    if( handle->blurMethod == AR2_CONSTANT_BLUR ) {
        if( ar2GetBestMatching( dataPtr,
                                mfImage,
                                handle->xsize,
                                handle->ysize,
                                handle->pixFormat,
                               *templ,
                                handle->searchSize,
                                handle->searchSize,
                                search,
                                &bx, &by,
                              &(result->sim)) < 0 ) {
            return -1;
        }
        result->blurLevel = handle->blurLevel;
    }
    else {
        if( ar2GetBestMatching2( dataPtr,
                                 mfImage,
                                 handle->xsize,
                                 handle->ysize,
                                 handle->pixFormat,
                                *templ2,
                                 handle->searchSize,
                                 handle->searchSize,
                                 search,
                                 &bx, &by,
                               &(result->sim),
                               &(result->blurLevel)) < 0 ) {
            return -1;
        }
    }
#else
    if( ar2GetBestMatching( dataPtr,
                            mfImage,
                            handle->xsize,
                            handle->ysize,
                            handle->pixFormat,
                           *templ,
                            handle->searchSize,
                            handle->searchSize,
                            search,
                            &bx, &by,
                          &(result->sim)) < 0 ) {
        return -1;
    }
#endif

    result->pos2d[0] = (float)bx;
    result->pos2d[1] = (float)by;
    result->pos3d[0] = surfaceSet->surface[snum].trans[0][0] * surfaceSet->surface[snum].featureSet->list[level].coord[fnum].mx
                     + surfaceSet->surface[snum].trans[0][1] * surfaceSet->surface[snum].featureSet->list[level].coord[fnum].my
                     + surfaceSet->surface[snum].trans[0][3];
    result->pos3d[1] = surfaceSet->surface[snum].trans[1][0] * surfaceSet->surface[snum].featureSet->list[level].coord[fnum].mx
                     + surfaceSet->surface[snum].trans[1][1] * surfaceSet->surface[snum].featureSet->list[level].coord[fnum].my
                     + surfaceSet->surface[snum].trans[1][3];
    result->pos3d[2] = surfaceSet->surface[snum].trans[2][0] * surfaceSet->surface[snum].featureSet->list[level].coord[fnum].mx
                     + surfaceSet->surface[snum].trans[2][1] * surfaceSet->surface[snum].featureSet->list[level].coord[fnum].my
                     + surfaceSet->surface[snum].trans[2][3];

    EM_ASM_({
      var a = arguments;
      var s = artoolkit.kimDebugMatching.tracking2dSub[artoolkit.kimDebugMatching.tracking2dSub.length-1];
      s.bestMatched.push({
        pos2d: ([a[0], a[1]]),
        pos3d: ([a[2], a[3], a[4]]),
        sim: a[5],
        mx: a[6],
        my: a[7],
        trans: ([
          [a[8], a[9], null, a[10]],
          [a[11], a[12], null, a[14]],
          [a[15], a[16], null, a[17]],
        ])
      });
    }, result->pos2d[0], result->pos2d[1], result->pos3d[0], result->pos3d[1], result->pos3d[2], result->sim,
      surfaceSet->surface[snum].featureSet->list[level].coord[fnum].mx,
      surfaceSet->surface[snum].featureSet->list[level].coord[fnum].my,
      surfaceSet->surface[snum].trans[0][0],
      surfaceSet->surface[snum].trans[0][1],
      surfaceSet->surface[snum].trans[0][3],
      surfaceSet->surface[snum].trans[1][0],
      surfaceSet->surface[snum].trans[1][1],
      surfaceSet->surface[snum].trans[1][3],
      surfaceSet->surface[snum].trans[1][0],
      surfaceSet->surface[snum].trans[1][1],
      surfaceSet->surface[snum].trans[1][3]
      ); 

    return 0;
}
