classdef Link < handle
    
    properties
        type
        mass
        a
        b
        c % width for prism, length for cylinder
        pli % CoM wrt base frame
        pj % previous joint origin wrt base frame
    end

    methods
        function obj = Link(type)
            obj.type = type;
        end
    end
end