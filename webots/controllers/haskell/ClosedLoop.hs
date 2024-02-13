module ClosedLoop where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State

type ClosedLoopController = State -> Demands -> Demands

