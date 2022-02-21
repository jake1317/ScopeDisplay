import xml.etree.ElementTree as ET
import re
import math
from PIL import Image
import numpy as np
import scipy.misc as smp
import wave
import random
import struct
import sys

def splitPathStringList(concatString):
    whitespacePattern = re.compile('\s+')
    subpathEndPattern = re.compile('[Zz]\s+')
    myCondensedWhitespace = whitespacePattern.sub(" ", concatString)
    myConcatString = subpathEndPattern.sub("z", myCondensedWhitespace)
    endPattern = re.compile('[Zz]')
    pathList = []
    while len(myConcatString) > 0:
        endSearch = endPattern.search(myConcatString)
        if endSearch:
            pathList.append(myConcatString[:endSearch.end()])
            myConcatString = myConcatString[endSearch.end():]
        else:
            pathList.append(myConcatString)
            return pathList
    return pathList

def getSvgPathStrings(fd):
    pathString = ""
    root = ET.parse(fd).getroot()
    for path in root.iter():
        if 'path' in path.tag:
            pathString += path.attrib['d']
    return splitPathStringList(pathString)

def getFloats(floatString):
    decimalCount = floatString.count('.')
    if decimalCount < 2:
        return [float(floatString)]
    if decimalCount >= 2:
        retList = []
        splitString = floatString.split('.')
        retList.append(float(splitString[0]+"."+splitString[1]))
        for decimal in splitString[2:]:
            retList.append(float("0."+decimal))
        return retList

def parseFloatList(floatString):
    floatList = []
    myFloatString = floatString
    delimitPattern = re.compile("[,-]")
    while len(myFloatString) > 0:
        searchStartIdx = 1 if myFloatString[0] == '-' else 0
        delimitMatch = delimitPattern.search(myFloatString[searchStartIdx:])
        if delimitMatch:
            matchStart = delimitMatch.start() + searchStartIdx
            floatList += getFloats(myFloatString[:matchStart])
            myFloatString = myFloatString[matchStart:]
            if myFloatString[0] == ',':
                myFloatString = myFloatString[1:]
        else:
            floatList += getFloats(myFloatString)
            return floatList

    return floatList

def getPathCommandList(pathString):
    if len(pathString) == 0:
        return []
    pathCommandList = []
    commandPattern = re.compile("[A-Za-z]")
    myPathString = pathString[1:].replace(' ', ',')
    command = pathString[0]

    while len(myPathString) > 0:
        if len(myPathString) == 1:
            pathCommandList.append((command, ""))
            break

        nextCommand = commandPattern.search(myPathString)
        if nextCommand:
            pathCommandList.append((command, parseFloatList(myPathString[:nextCommand.start()])))
            command = myPathString[nextCommand.start()]
            myPathString = myPathString[nextCommand.end():]
            if command == 'Z' or command == 'z':
                pathCommandList.append((command, ""))
        else:
            pathCommandList.append((command, parseFloatList(myPathString)))
            break
    return pathCommandList

def getDistance(start, end):
    return math.sqrt(((start[0] - end[0]) ** 2) + ((start[1] - end[1]) ** 2))

def getLine(start, end, rate):
    linePath = []
    distance = getDistance(start, end)
    numPoints = int(rate * distance) + 1
    slopeIncrement = ((end[0] - start[0])/numPoints, (end[1] - start[1])/numPoints)
    for i in range(1, numPoints):
        x = start[0] + (i * slopeIncrement[0])
        y = start[1] + (i * slopeIncrement[1])
        linePath.append((x, y))
    return linePath

def getBezierCoord(pointA, pointB, ratio):
    slopeX = pointB[0] - pointA[0]
    slopeY = pointB[1] - pointA[1]
    return (pointA[0] + (slopeX * ratio), pointA[1] + (slopeY * ratio))

def computeBezier(start, ctrlA, ctrlB, end, rate):
    myPath = []
    numPoints = int(rate * getDistance(start, end)) + 1
    for i in range(1, numPoints):
        ratio = i / float(numPoints)
        myPath.append(getBezierCoord(getBezierCoord(start, ctrlA, ratio), getBezierCoord(ctrlB, end, ratio), ratio))
    return myPath

def getCtrlAForShortBezier(prevCommand, prevCoordinates):
    command, floatList = prevCommand
    prevEnd = prevCoordinates
    prevCtrlB = prevCoordinates
    if command == 'C':
        prevEnd = (floatList[4], floatList[5])
        prevCtrlB = (floatList[2], floatList[3])
    elif command == 'S':
        prevEnd = (floatList[2], floatList[3])
        prevCtrlB = (floatList[0], floatList[1])
    elif command == 'c':
        prevCtrlB = (prevEnd[0] - floatList[4] + floatList[2], prevEnd[1] - floatList[5] + floatList[3]) #TODO: maybe this is a bug
    elif command == 's':
        prevCtrlB = (prevEnd[0] - floatList[2] + floatList[0], prevEnd[1] - floatList[3] + floatList[1]) #TODO: maybe this is a bug
    elif command == 'Q':
        prevEnd = (floatList[2], floatList[3])
        prevCtrlB = (floatList[0], floatList[1])
    elif command == 'T' or command == 't':
        raise Exception("Does not yet support consecutive short quadratic beziers!")
        prevEnd = (floatList[2], floatList[3])
        prevCtrlB = (floatList[0], floatList[1])
    elif command == 'q':
        raise Exception("Does not yet support relative quadratic beziers!")
    return ((2 * prevEnd[0]) - prevCtrlB[0], (2 * prevEnd[1]) - prevCtrlB[1])

def computeEllipseArc(start, radiusX, radiusY, rotation, largeArc, sweepFlag, end, rate):
    return getLine(start, end, rate) #TODO: actually compute this

def parseShortCubicBezier(prevPoint, prevCommand, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 4:
        ctrlA = getCtrlAForShortBezier(prevCommand, prevPoint)
        ctrlB = (floatList[0], floatList[1])
        end = (floatList[2], floatList[3])
        if isRelative:
            ctrlB = (prevPoint[0] + floatList[0], prevPoint[1] + floatList[1])
            end = (prevPoint[0] + floatList[2], prevPoint[1] + floatList[3])
        myPath.extend(computeBezier(prevPoint, ctrlA, ctrlB, end, lineRate))
        floatList = floatList[4:]
        prevPoint = end
        prevCommand = 'c'
    return myPath

def parseQuadraticBezier(prevPoint, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 4:
        ctrlPt = (floatList[0], floatList[1])
        end = (floatList[2], floatList[3])
        if isRelative:
            ctrlPt = (prevPoint[0] + floatList[0], prevPoint[1] + floatList[1])
            end = (prevPoint[0] + floatList[2], prevPoint[1] + floatList[3])
        myPath.extend(computeBezier(prevPoint, ctrlPt, ctrlPt, end, lineRate))
        floatList = floatList[4:]
        prevPoint = end
    return myPath

def parseShortQuadraticBezier(prevPoint, prevCommand, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 2:
        ctrlPt = getCtrlAForShortBezier(prevCommand, prevPoint)
        end = (floatList[0], floatList[1])
        if isRelative:
            end = (prevPoint[0] + floatList[0], prevPoint[1] + floatList[1])
        myPath.extend(computeBezier(prevPoint, ctrlPt, ctrlPt, end, lineRate))
        floatList = floatList[2:]
        prevPoint = end
        prevCommand = 'q'
    return myPath

def parseEllipticArc(prevPoint, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 7:
        end = (floatList[5], floatList[6])
        if isRelative:
            end = (prevPoint[0] + floatList[5], prevPoint[1] + floatList[6])
        largeArc = floatList[3] == 1
        sweepFlag = floatList[4] == 1
        myPath.extend(computeEllipseArc(prevPoint, floatList[0], floatList[1], floatList[2], largeArc, sweepFlag, end, lineRate))
        floatList = floatList[7:]
        prevPoint = end
    return myPath

def inflateMovetoCommands(prevPoint, command, floatList):
    myAbsoluteList = []
    first = True
    while len(floatList) >= 2:
        myCommand = 'M' if first else 'L'
        myFloatList = floatList[:2]
        if command == 'm':
            myFloatList[0] += prevPoint[0]
            myFloatList[1] += prevPoint[1]
        myAbsoluteList.append((myCommand, myFloatList))
        prevPoint = (myFloatList[0], myFloatList[1])
        floatList = floatList[2:]
        first = False
    return myAbsoluteList

def inflateLinetoCommands(prevPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 2:
        myFloatList = floatList[:2]
        if command == 'l':
            myFloatList[0] += prevPoint[0]
            myFloatList[1] += prevPoint[1]
        myAbsoluteList.append(('L', myFloatList))
        prevPoint = (myFloatList[0], myFloatList[1])
        floatList = floatList[2:]
    return myAbsoluteList

# H and V commands will be inflated into absolute lineto commands
def inflateHVCommands(prevPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 1:
        newX = prevPoint[0]
        newY = prevPoint[1]
        if command == 'H':
            newX = floatList[0]
        elif command == 'h':
            newX += floatList[0]
        elif command == 'V':
            newY = floatList[0]
        elif command == 'v':
            newY += floatList[0]
        myAbsoluteList.append(('L', [newX, newY]))
        prevpoint = (newX, newY)
        floatList = floatList[1:]
    return myAbsoluteList

def inflateLongCurvetoCommands(prevPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 6:
        myFloatList = floatList[:6]
        if command == 'c':
            myFloatList[0] += prevPoint[0]
            myFloatList[1] += prevPoint[1]
            myFloatList[2] += prevPoint[0]
            myFloatList[3] += prevPoint[1]
            myFloatList[4] += prevPoint[0]
            myFloatList[5] += prevPoint[1]
        myAbsoluteList.append(('C', myFloatList))
        prevPoint = (myFloatList[4], myFloatList[5])
        floatList = floatList[6:]
    return myAbsoluteList

def inflateShortCurvetoCommands(prevPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 4:
        myFloatList = floatList[:4]
        if command == 'c':
            myFloatList[0] += prevPoint[0]
            myFloatList[1] += prevPoint[1]
            myFloatList[2] += prevPoint[0]
            myFloatList[3] += prevPoint[1]
        myAbsoluteList.append(('C', myFloatList))
        prevPoint = (myFloatList[4], myFloatList[5])
        floatList = floatList[6:]
    return myAbsoluteList

def inflateCommandLists(commandLists):
    myInflatedCommandLists = []
    myCurrentInflatedCommandList = []
    prevPoint = (0, 0)
    initialPoint = (0, 0)
    for commandList in commandLists:
        myCurrentInflatedCommandList = []
        for fullCommand in commandList:
            command, floatList = fullCommand
            if command == 'M' or command == 'm':
                myInflatedCommands = inflateMovetoCommands(prevPoint, command, floatList)
                initialPoint = (myInflatedCommands[0][1][0], myInflatedCommands[0][1][1])
                prevPoint = (myInflatedCommands[-1][1][0], myInflatedCommands[-1][1][1])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'L' or command == 'l':
                myInflatedCommands = inflateLinetoCommands(prevPoint, command, floatList)
                prevPoint = (myInflatedCommands[-1][1][0], myInflatedCommands[-1][1][1])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'H' or command == 'h' or command == 'V' or command == 'v':
                myInflatedCommands = inflateHVCommands(prevPoint, command, floatList)
                prevPoint = (myInflatedCommands[-1][1][0], myInflatedCommands[-1][1][1])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'C' or command == 'c':
                myInflatedCommands = inflateLongCurvetoCommands(prevPoint, command, floatList)
                prevPoint = (myInflatedCommands[-1][1][0], myInflatedCommands[-1][1][1])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'S' or command == 's':
                myCurrentInflatedCommandList.append(fullCommand)
            elif command == 'Q' or command == 'q':
                myCurrentInflatedCommandList.append(fullCommand)
            elif command == 'T' or command == 't':
                myCurrentInflatedCommandList.append(fullCommand)
            elif command == 'A' or command == 'a':
                myCurrentInflatedCommandList.append(fullCommand)
            elif command == 'Z' or command == 'z':
                myCurrentInflatedCommandList.append(fullCommand)
            else:
                raise Exception("Unknown Command: " + command)

        myInflatedCommandLists.append(myCurrentInflatedCommandList)

    return myInflatedCommandLists


def getPathsFromCommandLists(commandLists, lineRate):
    myPaths = []
    myCurrentPath = []
    initialPoint = (0,0)
    for commandList in commandLists:
        myCurrentPath = []
        for i in range(len(commandList)):
            command, floatList = commandList[i]
            prevPoint = (0,0)
            if i > 0:
                prevPoint = myCurrentPath[-1]
            elif len(myPaths) > 0:
                prevPoint = myPaths[-1][-1]
            if command == 'M':
                initialPoint = (floatList[0], floatList[1])
                myCurrentPath.append(initialPoint)
            elif command == 'L':
                end = (floatList[0], floatList[1])
                myCurrentPath.extend(getLine(prevPoint, end, lineRate))
            elif command == 'C':
                ctrlA = (floatList[0], floatList[1])
                ctrlB = (floatList[2], floatList[3])
                end = (floatList[4], floatList[5])
                myCurrentPath.extend(computeBezier(prevPoint, ctrlA, ctrlB, end, lineRate))
            elif command == 'S':
                myCurrentPath.extend(parseShortCubicBezier(prevPoint, commandList[i-1], floatList, False, lineRate))
            elif command == 's':
                myCurrentPath.extend(parseShortCubicBezier(prevPoint, commandList[i-1], floatList, True, lineRate))
            elif command == 'Q':
                myCurrentPath.extend(parseQuadraticBezier(prevPoint, floatList, False, lineRate))
            elif command == 'q':
                myCurrentPath.extend(parseQuadraticBezier(prevPoint, floatList, True, lineRate))
            elif command == 'T':
                myCurrentPath.extend(parseShortQuadraticBezier(prevPoint, floatList, False, lineRate))
            elif command == 't':
                myCurrentPath.extend(parseShortQuadraticBezier(prevPoint, floatList, True, lineRate))
            elif command == 'A':
                myCurrentPath.extend(parseEllipticArc(prevPoint, floatList, False, lineRate))
            elif command == 'a':
                myCurrentPath.extend(parseEllipticArc(prevPoint, floatList, True, lineRate))
            elif command == 'Z' or command == 'z':
                myCurrentPath.extend(getLine(myCurrentPath[-1], initialPoint, lineRate))
            elif command == 'm' or command == 'l' or command == 'H' or command == 'h' or command == 'V' or command == 'v' or command == 'c':
                raise Exception("Should only get inflated commands!")
            else:
                raise Exception("Unknown Command: " + command)
        myPaths.append(myCurrentPath)
    return myPaths

def addConnector(end, start, path, lineRate):
    connectingLine = getLine(end, start, lineRate)
    if len(connectingLine) > 2:
        connectingLine = connectingLine[1:-1]
    path.extend(connectingLine)

def findShortestConnector(pathA, pathB, interval = 100):
    best = (pathA[0], pathB[0], getDistance(pathA[0], pathB[0]))
    for iA in range(int(len(pathA)/interval)):
        for iB in range(int(len(pathB)/interval)):
            distance = getDistance(pathA[iA * interval], pathB[iB * interval])
            if distance < best[2]:
                best = (pathA[iA * interval], pathB[iB * interval], distance)
    return best

def getPathCenter(path):
    xSum = 0;
    ySum = 0;
    for point in path:
        xSum += point[0]
        ySum += point[1]
    xAvg = float(xSum) / float(len(path))
    yAvg = float(ySum) / float(len(path))
    return (xAvg, yAvg)

def getPathCenters(paths):
    xSum = 0;
    ySum = 0;
    totalLength = 0
    for path in paths:
        for point in path:
            xSum += point[0]
            ySum += point[1]
        totalLength += len(path)
    xAvg = float(xSum) / float(totalLength)
    yAvg = float(ySum) / float(totalLength)
    return (xAvg, yAvg)

def connectPaths(paths, lineRate):
    myPath = paths[0]
    if len(paths) > 1:
        for i in range(1, len(paths)):
            best = findShortestConnector(paths[i-1], paths[i])
            addConnector(best[0], best[1], myPath, lineRate)
            myPath.extend(paths[i])

    return myPath

def drawAscii(paths):
    factor = 1
    canvas = [[' ' for i in range(250 * factor)] for j in range(100 * factor)]
    for path in paths:
        for coord in path:
            x = int(coord[0] * factor)
            y = int(coord[1] * factor)
            if y < len(canvas) and x < len(canvas[0]):
                canvas[y][x] = 'X'
    for row in canvas:
        for char in row:
            print(char, end='')
        print('')

def drawImage(paths):
    size = 2000
    factor = 2
    data = np.zeros((size,size,3), dtype=np.uint8)

    for path in paths:
        for coord in path:
            x = int(coord[0] * factor)
            y = int(coord[1] * factor)
            if y > 0 and y < size and x > 0 and x < size:
                data[y][x] = [255, 255, 255]

    img = Image.fromarray(data)
    img.save('output.png')

def produceWav(pathList):
    sound = wave.open('output.wav', 'wb')
    sound.setnchannels(2)
    sound.setsampwidth(2)
    sound.setframerate(44100) #is this frames per second?
    sound.setnframes(1)
    for paths in pathList:
        pathCenterOrig = getPathCenters(paths)
        offset = 170
        mult = 50
        pathCenter = pathCenterOrig
        dataList = []
        for path in paths:
            for coord in path:
                xVal = int((coord[0] - pathCenter[0]) * mult)
                yVal = int((coord[1] - pathCenter[1]) * -mult)
                dataList.append(struct.pack('<hh', xVal, yVal))

        for i in range(1000):
            for data in dataList:
                sound.writeframesraw(data)
    sound.close()

if len(sys.argv) < 2:
    raise Exception("Must provide svg path!")
myImg = sys.argv[1]
pathStrings = getSvgPathStrings(myImg)
commandLists = [getPathCommandList(path) for path in pathStrings]
myList = getPathsFromCommandLists(inflateCommandLists(commandLists), 2)
drawImage(myList)

#combined = [myList0]
#produceWav(combined)
