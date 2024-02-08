{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Mixers where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import Motors

type Mixer = Demands -> Motors

quadAPMixer :: Mixer
quadAPMixer demands = QuadMotors m1 m2 m3 m4
  where (t, r, p, y) = (getDemands demands)
        m1 = t - r + p  - y
        m2 = t - r - p  + y
        m3 = t + r + p  + y
        m4 = t + r - p  - y

quadCFMixer :: Mixer
quadCFMixer demands = QuadMotors m1 m2 m3 m4
  where (t, r, p, y) = (getDemands demands)
        m1 = t - r + p  + y
        m2 = t - r - p  - y
        m3 = t + r - p  + y
        m4 = t + r + p  - y
