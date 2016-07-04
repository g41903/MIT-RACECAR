#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int is_valid(size_t xlen, size_t ylen, size_t x, size_t y){
  return (y >= 0 && y < ylen && x >= 0 && x < xlen);
}

int8_t get_map_val(int8_t* map, size_t xlen, size_t ylen, size_t x, size_t y){
  int index = xlen*y+x;
  return map[index];
}

void calc_line(int8_t* map, size_t xlen, size_t ylen, size_t sx, size_t sy, size_t *tx, size_t *ty){
  // Bresenham raytracing
  int x0,y0,x1,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = sx;
  y0 = sy;
  x1 = *tx;
  y1 = *ty;

  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(!is_valid(xlen, ylen, x, y) || get_map_val(map, xlen, ylen, x, y) > 50){
      *tx = x;
      *ty = y;
      return;
    }
  }
  else
  {
    if(!is_valid(xlen, ylen, x, y) || get_map_val(map, xlen, ylen, x, y) > 50){
      *tx = x;
      *ty = y;
      return;
    }
  }

  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(!is_valid(xlen, ylen, x, y) || get_map_val(map, xlen, ylen, x, y) > 50){
	*tx = x;
	*ty = y;
	return;
      }
    }
    else
    {
      if(!is_valid(xlen, ylen, x, y) || get_map_val(map, xlen, ylen, x, y) > 50){
	*tx = x;
	*ty = y;
	return;
      }
    }
  }
}
