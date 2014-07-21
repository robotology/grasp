function p = save_CAD(F,V,color)
    p=patch('faces', F, 'vertices' ,V);
    set(p, 'facec', color);               % Set the face color (force it)
    set(p, 'EdgeColor','none'); 
end

