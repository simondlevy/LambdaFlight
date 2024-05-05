{-# LANGUAGE DataKinds        #-}
{-# LANGUAGE RebindableSyntax #-}

module MinEkf where

import Language.Copilot hiding(atan2, (!!))
import Copilot.Compile.C99

import Utils

prediction_rate = 100 :: SInt32
prediction_update_interval_msec = (div 1000  prediction_rate) :: SInt32

stream_now_msec :: SInt32
stream_now_msec = extern "stream_now_msec" Nothing

ekfStep :: (SInt32, SInt32, SBool)
ekfStep = (stream_now_msec, nextPredictionMsec, shouldPredict) where

  shouldPredict = stream_now_msec >= _nextPredictionMsec

  nextPredictionMsec = if _nextPredictionMsec == 0 
                       then stream_now_msec 
                       else if stream_now_msec >= _nextPredictionMsec
                       then stream_now_msec + prediction_update_interval_msec
                       else _nextPredictionMsec

  _nextPredictionMsec = [0] ++ nextPredictionMsec
