function node = createANode(x,E,t,visible,parent,p,g,h)
%CREATEANODE Creates a new node object

node = [];
node.x = x;
node.E = E;
node.t = t;
node.visible = visible;
node.parent = parent;

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

