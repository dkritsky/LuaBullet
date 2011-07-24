local bullet = require 'bullet'

world = bullet.World:new()

box = bullet.Shape:newBox(5,5,5)
body = bullet.Body:new(box, 10)

world:add(body)

local i = 0
while i < 100 do
	i = i+1
	world:step(1/30.0)
	print(i, '=', body.position)
end
