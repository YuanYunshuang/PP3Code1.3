function [ image_addedBuffer ] = addBuffer( image, bufferSize)
[h,w]=size(image);
image_addedBuffer = zeros(h,w);
for i=bufferSize+1:h-bufferSize
    for j=bufferSize+1:w-bufferSize
        if image(i,j)>0
            for m=-bufferSize:bufferSize
                for n=-bufferSize:bufferSize
                    image_addedBuffer(i+m,j+n) = 1;
                end
            end
        end
    end
end

end

