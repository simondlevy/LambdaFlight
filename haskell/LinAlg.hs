module Main where

import Linear.Matrix

type Vector = [Float]

type Matrix = [Vector]

msq :: Matrix -> Matrix
msq a = a !*! a


printvec :: Vector -> IO()
printvec x = do
    print $ x

printmat' :: Matrix -> Int -> IO()
printmat' a rows = do
    printvec $ a!!0
    putStrLn ""
    printvec $ a!!1
    putStrLn ""
    printvec $ a!!2
    putStrLn ""
    printvec $ a!!3
    putStrLn ""
    printvec $ a!!4

 
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
