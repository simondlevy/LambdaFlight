module Main where

import Linear.Matrix hiding(transpose)
import Data.List

type Vector = [Float]

type Matrix = [Vector]

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

dot :: Vector -> Vector -> Float
dot (x:xs) (y:ys) = x * y + (dot xs ys)
dot [] [] = 0


main = do

    let a = [ [ 2,   3,  5],
              [ 7,  11, 13],
              [ 17, 19, 23 ] ]

    let x = [29, 31, 37]

    print $ dot x x

    -- printvec $ a !* x
    -- printmat $ transpose a
