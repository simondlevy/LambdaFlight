module Main where

import Linear.Matrix

type Vector = [Float]

type Matrix = [Vector]

msq :: Matrix -> Matrix
msq a = a !*! a


printvec' :: Vector -> Int -> IO()
printvec' _ 0 = do
  putStrLn ""  
printvec' x cols = do
    putStr . show $ head x
    putStr " "
    printvec' (tail x) (cols - 1)


printvec :: Vector -> IO()
printvec x = do
    printvec' x (length x)

printmat' :: Matrix -> Int -> IO()
printmat' _ 0 = do
  putStrLn ""  
printmat' a rows = do
    printvec $ head a
    printmat' (tail a) (rows - 1)

 
printmat :: Matrix -> IO()
printmat a = do
    printmat' a (length a)

main = do

    let a = [ [ 0,1,2,3,4 ],
              [ 1,2,3,4,0 ],
              [ 2,3,4,0,1 ],
              [ 3,4,0,1,2 ],
              [ 4,0,1,2,3 ] ]

    printmat $ msq a
