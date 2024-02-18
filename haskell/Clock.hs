{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Clock where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data ClockRate = 
              RATE_25_HZ
            | RATE_30_HZ
            | RATE_33_HZ
            | RATE_50_HZ
            | RATE_100_HZ
            | RATE_250_HZ
            | RATE_500_HZ
            | RATE_1000_HZ

rateToPeriod :: ClockRate -> SFloat
rateToPeriod rate = 1 / (rateToFloat rate)

rateToFloat :: ClockRate -> SFloat
rateToFloat RATE_25_HZ   = 25
rateToFloat RATE_30_HZ   = 30
rateToFloat RATE_33_HZ   = 33
rateToFloat RATE_50_HZ   = 50
rateToFloat RATE_100_HZ  = 100
rateToFloat RATE_250_HZ  = 250
rateToFloat RATE_500_HZ  = 500
rateToFloat RATE_1000_HZ = 1000

