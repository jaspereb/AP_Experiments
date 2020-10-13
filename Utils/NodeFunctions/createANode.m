function node = createANode(x,Sigma,parent,visible,p,g,h)
%CREATEANODE Creates a new node object

node = [];
node.x = x;
node.Sigma = Sigma;
node.parent = parent; %Another node object

if(exist('visible','var')) %Is the target visible from the current node
    node.visible = visible;
else
    node.visible = true;
end

if(exist('p','var'))
    node.p = p;
end

if(exist('g','var'))
    node.g = g;
end

if(exist('h','var'))
    node.h = h;
end


end

