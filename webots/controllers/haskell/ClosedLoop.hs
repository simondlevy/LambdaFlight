module ClosedLoop where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

type ClosedLoopController = SFloat -> State -> Demands -> Demands

