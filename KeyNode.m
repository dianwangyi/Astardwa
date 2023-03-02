function [keypath]  = KeyNode(path)

if length(path(: , 1)) <= 3
    keypath = path;
    return;
end

keypath = [path(1, :)];

firstNode  = 1;
midNode = 2;
lastNode = 3;

nodeCount = 1;

for keyIdx = 3 : length(path(:, 1))
    if ( ( path(midNode, 1) - path(firstNode, 1) ) == ( path(lastNode, 1) - path(midNode, 1) )  &&  ...
             ( path(midNode, 2) - path(firstNode, 2) ) == ( path(lastNode, 2) - path(midNode, 2) ) )
         nodeCount = nodeCount + 1;
         if nodeCount > 2
            keypath = [keypath; path(midNode , : ) ];
            firstNode = firstNode+1;
            midNode = midNode+1;
            lastNode = lastNode+1;
            nodeCount = 1;
         end
         continue;
    end
    keypath = [keypath; path(midNode , : ) ];
    firstNode = firstNode+1;
    midNode = midNode+1;
    lastNode = lastNode+1;
end
keypath = [keypath; path(length(path(:, 1)), : ) ];
