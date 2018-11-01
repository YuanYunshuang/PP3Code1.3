
result = zeros(height_ref, width_ref,3);
result(:,:,1) = img_ref;
result(:,:,2) = img_ref;
result(:,:,3) = img_ref;
I = double(imrotate(img_reg, -rot,'bilinear','crop'));
result(t_x:t_x+height-1,t_y:t_y+width-1,2) = result(t_x:(t_x+height-1),t_y:(t_y+width-1),2)+I;

imshow(result);