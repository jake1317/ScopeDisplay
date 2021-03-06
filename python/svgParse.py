import xml.etree.ElementTree as ET
import re
import math
from PIL import Image
import numpy as np
import scipy.misc as smp
import wave
import random
import struct

def splitPathStringList(concatString):
	endPattern = re.compile('[Zz]')
	myConcatString = concatString.replace(" ", "")
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
	for path in ET.parse(fd).getroot()[0]:
		pathString += path.attrib['d']	
	return splitPathStringList(pathString)

def parseFloatList(floatString):
	floatList = []
	myFloatString = floatString
	delimitPattern = re.compile("[,-]")
	while len(myFloatString) > 0:
		searchStartIdx = 1 if myFloatString[0] == '-' else 0
		delimitMatch = delimitPattern.search(myFloatString[searchStartIdx:])
		if delimitMatch:
			matchStart = delimitMatch.start() + searchStartIdx
			floatList.append(float(myFloatString[:matchStart]))	
			myFloatString = myFloatString[matchStart:]
			if myFloatString[0] == ',':
				myFloatString = myFloatString[1:]
		else:
			floatList.append(float(myFloatString))
			return floatList
	
	return floatList

def getPathCommandList(pathString):
	if len(pathString) == 0:
		return []
	pathCommandList = []
	commandPattern = re.compile("[A-Za-z]")
	myPathString = pathString[1:]
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
		prevCtrlB = (prevEnd[0] - floatList[4] + floatList[2], prevEnd[1] - floatList[5] + floatList[3])
	elif command == 's':
		prevCtrlB = (prevEnd[0] - floatList[2] + floatList[0], prevEnd[1] - floatList[3] + floatList[1])
	return ((2 * prevEnd[0]) - prevCtrlB[0], (2 * prevEnd[1]) - prevCtrlB[1])

def getPathFromCommandList(commandList, lineRate):
	myPath = []
	for i in range(len(commandList)):
		command, floatList = commandList[i]
		prevX, prevY = myPath[-1] if i > 0 else (0,0)
		if command == 'M':
			myPath.append((floatList[0], floatList[1]))
		elif command == 'm': # should actually be a new path, but don't want to deal rn
			myPath.append(((prevX + floatList[0]), (prevY + floatList[1])))
		elif command == 'L':
			end = (floatList[0], floatList[1])
			myPath.extend(getLine(myPath[-1], end, lineRate))
		elif command == 'l':
			end = ((prevX + floatList[0]), (prevY + floatList[1]))
			myPath.extend(getLine(myPath[-1], end, lineRate))
		elif command == 'H':
			end = (floatList[0], prevY)
			myPath.extend(getLine(myPath[-1], end, lineRate))
		elif command == 'h':
			end = ((prevX + floatList[0]), prevY)
			myPath.extend(getLine(myPath[-1], end, lineRate))
		elif command == 'V':
			end = (prevX, floatList[0])
			myPath.extend(getLine(myPath[-1], end, lineRate))
		elif command == 'v':
			end = (prevX, prevY + floatList[0])
			myPath.extend(getLine(myPath[-1], end, lineRate))
		elif command == 'C':
			start = (prevX, prevY)
			ctrlA = (floatList[0], floatList[1])
			ctrlB = (floatList[2], floatList[3])
			end = (floatList[4], floatList[5])
			myPath.extend(computeBezier(start, ctrlA, ctrlB, end, lineRate))
		elif command == 'c':
			start = (prevX, prevY)
			ctrlA = (prevX + floatList[0], prevY + floatList[1])
			ctrlB = (prevX + floatList[2], prevY + floatList[3])
			end = (prevX + floatList[4], prevY + floatList[5])
			myPath.extend(computeBezier(start, ctrlA, ctrlB, end, lineRate))
		elif command == 'S':
			start = (prevX, prevY)
			ctrlA = getCtrlAForShortBezier(commandList[i-1], myPath[-1])
			ctrlB = (floatList[0], floatList[1])
			end = (floatList[2], floatList[3])
			myPath.extend(computeBezier(start, ctrlA, ctrlB, end, lineRate))
		elif command == 's':
			start = (prevX, prevY)
			ctrlA = getCtrlAForShortBezier(commandList[i-1], myPath[-1])
			ctrlB = (prevX + floatList[0], prevY + floatList[1])
			end = (prevX + floatList[2], prevY + floatList[3])
			myPath.extend(computeBezier(start, ctrlA, ctrlB, end, lineRate))
		else:
			print("sucks idk what it is! ", command)
			return []
	return myPath

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
	size = 1000
	factor = 10
	data = np.zeros((2000,2000,3), dtype=np.uint8)

	for path in paths:
		for coord in path:
			x = int(coord[0] * factor)
			y = int(coord[1] * factor)
			if y < len(data) and x < len(data[0]):
				data[y][x] = [255, 255, 255]

	img = Image.fromarray(data)
	img.save('output.png')

def produceWav(paths):
	sound = wave.open('output.wav', 'wb')
	sound.setnchannels(2)
	sound.setsampwidth(2)
	sound.setframerate(44100) #is this frames per second?
	sound.setnframes(1)
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

pathStrings = getSvgPathStrings('friend2-01.svg')
commandLists = [getPathCommandList(path) for path in pathStrings]
myList = [getPathFromCommandList(commandList, 1) for commandList in commandLists]
#drawImage([connectPaths(myList, 1)])
produceWav(myList)
