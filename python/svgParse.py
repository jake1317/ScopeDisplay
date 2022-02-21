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
    endPattern = re.compile('[Zz]')
    pathList = []
    while len(concatString) > 0:
        endSearch = endPattern.search(concatString)
        if endSearch:
            pathList.append(concatString[:endSearch.end()])
            concatString = concatString[endSearch.end():]
        else:
            pathList.append(concatString)
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

def parseLCommand(prevPoint, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 2:
        end = (floatList[0], floatList[1])
        if isRelative:
            end = (prevPoint[0] + floatList[0], prevPoint[1] + floatList[1])
        myPath.extend(getLine(prevPoint, end, lineRate))
        floatList = floatList[2:]
        prevPoint = end
    return myPath

def parseHCommand(prevPoint, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 1:
        end = (floatList[0], prevPoint[1])
        if isRelative:
            end = (prevPoint[0] + floatList[0], prevPoint[1])
        myPath.extend(getLine(prevPoint, end, lineRate))
        floatList = floatList[1:]
        prevPoint = end
    return myPath

def parseVCommand(prevPoint, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 1:
        end = (prevPoint[0], floatList[0])
        if isRelative:
            end = (prevPoint[0], prevPoint[1] + floatList[0])
        myPath.extend(getLine(prevPoint, end, lineRate))
        floatList = floatList[1:]
        prevPoint = end
    return myPath

def parseCubicBezier(prevPoint, floatList, isRelative, lineRate):
    myPath = []
    while len(floatList) >= 6:
        ctrlA = (floatList[0], floatList[1])
        ctrlB = (floatList[2], floatList[3])
        end = (floatList[4], floatList[5])
        if isRelative:
            ctrlA = (prevPoint[0] + floatList[0], prevPoint[1] + floatList[1])
            ctrlB = (prevPoint[0] + floatList[2], prevPoint[1] + floatList[3])
            end = (prevPoint[0] + floatList[4], prevPoint[1] + floatList[5])
        myPath.extend(computeBezier(prevPoint, ctrlA, ctrlB, end, lineRate))
        floatList = floatList[6:]
        prevPoint = end
    return myPath

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
            elif command == 'm':
                myPaths.append(myCurrentPath)
                myCurrentPath = []
                initialPoint = (prevPoint[0] + floatList[0], prevPoint[1] + floatList[1])
                myCurrentPath.append(initialPoint)
            elif command == 'L':
                myCurrentPath.extend(parseLCommand(prevPoint, floatList, False, lineRate))
            elif command == 'l':
                myCurrentPath.extend(parseLCommand(prevPoint, floatList, True, lineRate))
            elif command == 'H':
                myCurrentPath.extend(parseHCommand(prevPoint, floatList, False, lineRate))
            elif command == 'h':
                myCurrentPath.extend(parseHCommand(prevPoint, floatList, True, lineRate))
            elif command == 'V':
                myCurrentPath.extend(parseVCommand(prevPoint, floatList, False, lineRate))
            elif command == 'v':
                myCurrentPath.extend(parseVCommand(prevPoint, floatList, True, lineRate))
            elif command == 'C':
                myCurrentPath.extend(parseCubicBezier(prevPoint, floatList, False, lineRate))
            elif command == 'c':
                myCurrentPath.extend(parseCubicBezier(prevPoint, floatList, True, lineRate))
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
pathStrings0 = getSvgPathStrings(myImg)
commandLists0 = [getPathCommandList(path) for path in pathStrings0]
myList = getPathsFromCommandLists(commandLists0, 2)
drawImage(myList)

#pathStrings1 = getSvgPathStrings('../images/test-10.svg')
#commandLists1 = [getPathCommandList(path) for path in pathStrings1]
#myList1 = [getPathFromCommandList(commandList, .5) for commandList in commandLists1]

#pathStrings2 = getSvgPathStrings('../images/002.svg')
#commandLists2 = [getPathCommandList(path) for path in pathStrings2]
#myList2 = [getPathFromCommandList(commandList, .5) for commandList in commandLists2]

#combined = [myList0]
#produceWav(combined)
