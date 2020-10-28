function node = createANode(x,y,Sigma,Cost,parent,visible,p,g,h)
%CREATEANODE Creates a new node object

node = [];
node.x = x; %Camera pose
node.y = y; %Target pose estimate
node.Sigma = Sigma;
node.parent = parent; %Another node object
node.Cost = Cost; %This is either tr(Sigma), tr(Weight*Sigma) or an arbitrary cost fn

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

