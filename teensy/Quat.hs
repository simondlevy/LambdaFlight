{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Quat where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding(atan2, (++))
import Utils

madgwick6DOF :: SFloat -> (SFloat, SFloat, SFloat)

madgwick6DOF gx = (phi, theta, psi) where

  q0 = q0' * gx

  q0' = [0] ++ q0

  phi = q0

  theta = 0

  psi = 0
