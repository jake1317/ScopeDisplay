import xml.etree.ElementTree as ET
import re
import math
from PIL import Image
from math import sqrt, cos, sin, acos, degrees, radians, log, pi
import numpy as np
import scipy.misc as smp
import wave
import random
import struct
from os import walk
import argparse

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
    slopeIncrement = ((end[0] - start[0])/float(numPoints), (end[1] - start[1])/float(numPoints))
    for i in range(1, numPoints + 1):
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
    for i in range(1, numPoints + 1):
        ratio = i / float(numPoints)
        myPath.append(getBezierCoord(getBezierCoord(start, ctrlA, ratio), getBezierCoord(ctrlB, end, ratio), ratio))
    return myPath



# This math is heavily borrowed from https://github.com/regebro/svg.path
def computeEllipseArc(start, radiusX, radiusY, rotation, largeArc, sweepFlag, end, rate):
    if start == end:
        return []
    if radiusX == 0 or radiusY == 0:
        return getLine(start, end, rate)
    rotation = rotation % 360

    cosr = cos(radians(rotation))
    sinr = sin(radians(rotation))
    deltaX = (start[0] - end[0]) / 2
    deltaY = (start[1] - end[1]) / 2
    x1prime = cosr * deltaX + sinr * deltaY
    x1prime_squared = x1prime * x1prime
    y1prime = -sinr * deltaX + cosr * deltaY
    y1prime_squared = y1prime * y1prime
    radiusX_squared = radiusX * radiusX
    radiusY_squared = radiusY * radiusY

    # Correct out of range radii
    radius_scale = (x1prime_squared / radiusX_squared) + (y1prime_squared / radiusY_squared)
    if radius_scale > 1:
        radius_scale = sqrt(radius_scale)
        radiusX *= radius_scale
        radiusY *= radius_scale
        radiusX_squared = radiusX * radiusX
        radiusY_squared = radiusY * radiusY
    else:
        radius_scale = 1

    t1 = radiusX_squared * y1prime_squared
    t2 = radiusY_squared * x1prime_squared
    c = sqrt(abs((radiusX_squared * radiusY_squared - t1 - t2) / (t1 + t2)))

    if largeArc == sweepFlag:
        c = -c

    cxprime = c * radiusX * y1prime / radiusY
    cyprime = -c * radiusY * x1prime / radiusX
    center = ((cosr * cxprime - sinr * cyprime) + ((start[0] + end[0]) / 2),
              (sinr * cxprime + cosr * cyprime) + ((start[1] + end[1]) / 2))

    ux = (x1prime - cxprime) / radiusX
    uy = (y1prime - cyprime) / radiusY
    vx = (-x1prime - cxprime) / radiusX
    vy = (-y1prime - cyprime) / radiusY
    n = sqrt(ux * ux + uy * uy)
    p = ux
    theta = degrees(acos(p / n))
    if uy < 0:
        theta = -theta
    theta = theta % 360

    n = sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy))
    p = ux * vx + uy * vy
    d = p / n
    if d > 1.0:
        d = 1.0
    elif d < -1.0:
        d = -1.0

    delta = degrees(acos(d))
    if (ux * vy - uy * vx) < 0:
        delta = -delta
    delta = delta % 360
    if not sweepFlag:
        delta -= 360

    length = getDistance(start, end) # Major oversimlification
    numPoints = int(rate * getDistance(start, end)) + 1

    myPath = []
    for i in range(1, numPoints + 1):
        pos = i / float(numPoints)
        angle = radians(theta + (delta * pos))
        x = (cosr * cos(angle) * radiusX) - (sinr * sin(angle) * radiusY) + center[0]
        y = (sinr * cos(angle) * radiusX) + (cosr * sin(angle) * radiusY) + center[1]
        myPath.append((x,y))

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

def inflateShortCurvetoCommands(prevPoint, prevCtrlPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 4:
        myFloatList = floatList[:4]
        if command == 's':
            myFloatList[0] += prevPoint[0]
            myFloatList[1] += prevPoint[1]
            myFloatList[2] += prevPoint[0]
            myFloatList[3] += prevPoint[1]
        myCtrlA = [(2 * prevPoint[0]) - prevCtrlPoint[0], (2 * prevPoint[1]) - prevCtrlPoint[1]]
        myFloatList = myCtrlA + myFloatList
        myAbsoluteList.append(('C', myFloatList))
        prevPoint = (myFloatList[4], myFloatList[5])
        prevCtrlPoint = (myFloatList[2], myFloatList[3])
        floatList = floatList[4:]
    return myAbsoluteList

def inflateLongQuadraticCurvetoCommands(prevPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 4:
        myFloatList = floatList[:4]
        if command == 'q':
            myFloatList[0] += prevPoint[0]
            myFloatList[1] += prevPoint[1]
            myFloatList[2] += prevPoint[0]
            myFloatList[3] += prevPoint[1]
        myFloatList = myFloatList[:2] + myFloatList
        myAbsoluteList.append(('C', myFloatList))
        prevPoint = (myFloatList[4], myFloatList[5])
        floatList = floatList[4:]
    return myAbsoluteList

def inflateShortQuadraticCurvetoCommands(prevPoint, prevCtrlPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 2:
        myFloatList = floatList[:2]
        if command == 't':
            myFloatList[0] += prevPoint[0]
            myFloatList[1] += prevPoint[1]
        myCtrlA = [(2 * prevPoint[0]) - prevCtrlPoint[0], (2 * prevPoint[1]) - prevCtrlPoint[1]]
        myFloatList = myCtrlA + myCtrlA + myFloatList
        myAbsoluteList.append(('C', myFloatList))
        prevPoint = (myFloatList[4], myFloatList[5])
        prevCtrlPoint = (myFloatList[2], myFloatList[3])
        floatList = floatList[2:]
    return myAbsoluteList

def inflateEllipseArcCommands(prevPoint, command, floatList):
    myAbsoluteList = []
    while len(floatList) >= 7:
        myFloatList = floatList[:7]
        if command == 'a':
            myFloatList[5] += prevPoint[0]
            myFloatList[6] += prevPoint[1]
        myAbsoluteList.append(('A', myFloatList))
        prevPoint = (myFloatList[5], myFloatList[6])
        floatList = floatList[7:]
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
                prevPoint = (myInflatedCommands[-1][1][4], myInflatedCommands[-1][1][5])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'S' or command == 's':
                myPrevCommand = myCurrentInflatedCommandList[-1]
                prevCtrlPoint = (myPrevCommand[1][2], myPrevCommand[1][3]) if myPrevCommand[0] == 'C' else prevPoint
                myInflatedCommands = inflateShortCurvetoCommands(prevPoint, prevCtrlPoint, command, floatList)
                prevPoint = (myInflatedCommands[-1][1][4], myInflatedCommands[-1][1][5])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'Q' or command == 'q':
                myInflatedCommands = inflateLongQuadraticCurvetoCommands(prevPoint, command, floatList)
                prevPoint = (myInflatedCommands[-1][1][4], myInflatedCommands[-1][1][5])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'T' or command == 't':
                myPrevCommand = myCurrentInflatedCommandList[-1]
                prevCtrlPoint = (myPrevCommand[1][2], myPrevCommand[1][3]) if myPrevCommand[0] == 'C' else prevPoint
                myInflatedCommands = inflateShortQuadraticCurvetoCommands(prevPoint, prevCtrlPoint, command, floatList)
                prevPoint = (myInflatedCommands[-1][1][4], myInflatedCommands[-1][1][5])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'A' or command == 'a':
                myInflatedCommands = inflateEllipseArcCommands(prevPoint, command, floatList)
                prevPoint = (myInflatedCommands[-1][1][5], myInflatedCommands[-1][1][6])
                myCurrentInflatedCommandList.extend(myInflatedCommands)
            elif command == 'Z' or command == 'z':
                myCurrentInflatedCommandList.append(('Z', []))
                prevPoint = initialPoint
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
            elif command == 'A':
                largeArc = floatList[3] != 0
                sweepFlag = floatList[4] != 0
                end = (floatList[5], floatList[6])
                myCurrentPath.extend(computeEllipseArc(prevPoint, floatList[0], floatList[1], floatList[2], largeArc, sweepFlag, end, lineRate))
            elif command == 'Z':
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

def drawImage(paths, targetFile):
    if len(paths) == 0:
        return

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
    img.save(targetFile + '.png')

def produceWav(pathList, targetFile, frameLoops, animationLoops, amplitude):
    sound = wave.open(targetFile + '.wav', 'wb')
    sound.setnchannels(2)
    sound.setsampwidth(2)
    sound.setframerate(44100) #is this frames per second?
    sound.setnframes(1)
    for outerLoop in range(animationLoops):
        for paths in pathList:
            pathCenterOrig = getPathCenters(paths)
            amplitude = 50
            pathCenter = pathCenterOrig
            dataList = []
            for path in paths:
                for coord in path:
                    xVal = int((coord[0] - pathCenter[0]) * amplitude)
                    yVal = int((coord[1] - pathCenter[1]) * -amplitude)
                    dataList.append(struct.pack('<hh', xVal, yVal))

            for i in range(frameLoops):
                for data in dataList:
                    sound.writeframesraw(data)
    sound.close()

def getFilesFromDir(dirPath):
    svgPaths = []
    numPattern = re.compile("[0-9]+")
    for (dirpath, dirnames, filenames) in walk(dirPath):
        for file in filenames:
            if file[-4:] != '.svg':
                continue
            numSearch = numPattern.search(file)
            fileNum = int(numSearch[0]) if numSearch else 0
            svgPaths.append(((dirpath+'/'+file), file, fileNum))
    return [(tup[0], tup[1]) for tup in sorted(svgPaths, key= lambda x: x[2])]

def stripFileExtention(file):
    for i in range(len(file)-1, -1, -1):
        if file[i] == '.':
            return file[:i]
    return file

def parseArgs():
    parser = argparse.ArgumentParser(description="Svg parser arguments")
    parser.add_argument('--svgdir', type=str, nargs=1, required=True, help='Path to directory containing svgs')
    parser.add_argument('--animate', action='store_true', help='Treat each svg as frame in animation, will order by number in filename')
    parser.add_argument('--rate', type=float, nargs=1, default=1, help='Rate at which paths will be drawn')
    parser.add_argument('--amplitude', type=float, nargs=1, default=50, help='Coefficient to apply to waveform magnitudes')
    parser.add_argument('--frameloops', type=int, nargs=1, default=1, help='Number of times each frame path is looped')
    parser.add_argument('--animationloops', type=int, nargs=1, default=1, help='Treat each svg as frame in animation, will order by number in filename')
    parser.add_argument('--png', action='store_true', help='In addition to audio output, produce image of path')
    parser.add_argument('--targetdir', type=str, nargs=1, default='', help='Path to directory of output files')
    return parser.parse_args()

def main():
    args = parseArgs()
    files = getFilesFromDir(args.svgdir[0])
    pathList = []
    if len(files) == 0:
        raise Exception("Could not find any files in specified path!!")
    targetDir = ''
    if len(args.targetdir) != 0:
        targetDir = args.targetdir[0]
        if targetDir[-1] != '/':
            targetDir += '/'
    frameLoops = 1
    if args.frameloops != 1:
        frameLoops = args.frameloops[0]
    animationLoops = 1
    if args.animationloops != 1:
        animationLoops = args.animationloops[0]
    for (fullPath, fileName) in files:
        pathStrings = getSvgPathStrings(fullPath)
        if len(pathStrings) == 0:
            raise Exception("Could not find any valid paths in SVG! " + fileName)
        commandLists = [getPathCommandList(path) for path in pathStrings]
        myPaths = getPathsFromCommandLists(inflateCommandLists(commandLists), args.rate)
        targetFile = targetDir + stripFileExtention(fileName)
        if args.png:
            drawImage(myPaths, targetFile)
        if args.animate:
            pathList.append(myPaths)
        else:
            produceWav([myPaths], targetFile, frameLoops, animationLoops, args.amplitude)
    if len(pathList) > 0:
        targetFile = targetDir + 'animation'
        produceWav([myPaths], targetFile, frameLoops, animationLoops, args.amplitude)

main()
