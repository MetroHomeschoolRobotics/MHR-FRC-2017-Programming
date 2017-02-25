list = GetArrayVariable("LINE_PATTERN")


SetVariable "X1", 0
SetVariable "Y1", 0
SetVariable "X2", 0
SetVariable "Y2", 0
SetVariable "X3", 0
SetVariable "Y3", 0
SetVariable "X4", 0
SetVariable "Y4", 0

if isArray(list) then
  if ubound(list) > 0 then

		' based on a line pattern in 8 x inches 
		' of 10 17 31 32 10 = 100
	
    targetPixelHeight = (list(3)/10)
    targetSamples = 0

    ' calibrated for an Axis camera
    imageHeight = GetVariable("IMAGE_HEIGHT")
    cameraFieldOfView = 47.5
    targetHeight = 100.0

		' determine distance in 8 x inches
    totalDistance = (((targetHeight*imageHeight)/targetPixelHeight)/2)/_
      tan(((cameraFieldOfView*3.14159)/180.0)/2.0)

		' convert to ft (12 inch per ft * 8 inch multiplier) = 96
		totalDistance = CInt((totalDistance*100)/96)/100

		' save it for use in other modules
    SetVariable "Distance", totalDistance

		widthDiff = list(6) - list(4)
		heightDiff = list(7) - list(5)
		
		SetVariable "X1", list(1) - widthDiff
		SetVariable "Y1", list(2) + heightDiff
		
		SetVariable "X2", list(1) + widthDiff
		SetVariable "Y2", list(2) + heightDiff
		
		SetVariable "X3", list(1) + widthDiff
		SetVariable "Y3", list(2) - heightDiff
		
		SetVariable "X4", list(1) - widthDiff
		SetVariable "Y4", list(2) - heightDiff
		
  end if
end if		
