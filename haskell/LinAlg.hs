module Main where

import Linear.Matrix hiding(transpose)
import Data.List

type Vector = [Float]

type Matrix = [Vector]

msq :: Matrix -> Matrix
msq a = a !*! a

printvec' :: Vector -> Int -> IO()
printvec' _ 0 = do
  putStrLn "\n"  
printvec' x cols = do
    putStr . show $ head x
    putStr " "
    printvec' (tail x) (cols - 1)


printvec :: Vector -> IO()
printvec x = do
    printvec' x (length x)

printmat' :: Matrix -> Int -> IO()
printmat' _ 0 = do
  putStr ""  
printmat' a rows = do
    printvec $ head a
    printmat' (tail a) (rows - 1)

 
printmat :: Matrix -> IO()
printmat a = do
    printmat' a (length a)

-- https://stackoverflow.com/questions/34423279/rotate-a-matrix-in-haskell
transp :: Matrix -> Matrix
transp = transpose . map reverse

main = do

    let a = [ [ 0,1,2],
              [ 3,4,5],
              [ 6,7,8 ] ]

    let b = transpose a

    printmat $ a
    print "---------------------"
    printmat $ b
