
#include "localizecore.h"

#define      HUGE_DISTANCE     32000

/* compute minimum distance to all occupied cells */

static void create_distance_map(map_p cmap, localize_map_p lmap,
				localize_param_p param)
{
  int x, y, i, j, border;
  double v;

  for(x = 0; x < lmap->config.x_size; x++)
    for(y = 0; y < lmap->config.y_size; y++) {
      lmap->distance[x][y] = HUGE_DISTANCE;
      lmap->x_offset[x][y] = HUGE_DISTANCE;
      lmap->y_offset[x][y] = HUGE_DISTANCE;
    }
  
  /* Initialize the distance measurements before dynamic programming */
  for(x = 0; x < lmap->config.x_size; x++)
    for(y = 0; y < lmap->config.y_size; y++)
      if(cmap->map[x][y] > param->occupied_prob) {
		  border = 0;
		  for(i = -1; i <= 1; i++)
			  for(j = -1; j <= 1; j++)
				  if(!border && x + i >= 0 && y + j >= 0 && x + i < lmap->config.x_size &&  y + j < lmap->config.y_size && (i != 0 || j != 0)) {
					  if(cmap->map[x + i][y + j] < param->occupied_prob && cmap->map[x + i][y + j] != -1)
						  border = 1;
				  }
				  if(border) {
					  lmap->distance[x][y] = 0;
					  lmap->x_offset[x][y] = 0;
					  lmap->y_offset[x][y] = 0;
				  }
	  }

	  /* Use dynamic programming to estimate the minimum distnance from
	  every map cell to an occupied map cell */

	  /* pass 1 */
	  for(x = 0; x < lmap->config.x_size; x++)
		  for(y = 0; y < lmap->config.y_size; y++)
			  for(i = -1; i <= 1; i++)
				  for(j = -1; j <= 1; j++) 
					  if(x + i >= 0 && y + j >= 0 && x + i < lmap->config.x_size && y + j < lmap->config.y_size && (i != 0 || j != 0)) {
						  v = lmap->distance[x + i][y + j] + ((i * j != 0) ? 1.414 : 1);
						  if(v < lmap->distance[x][y]) {
							  lmap->distance[x][y] = v;
							  lmap->x_offset[x][y] = lmap->x_offset[x + i][y + j] + i;
							  lmap->y_offset[x][y] = lmap->y_offset[x + i][y + j] + j;
						  }
					  }

					  /* pass 2 */
					  for(x = lmap->config.x_size - 1; x >= 0; x--)
						  for(y = lmap->config.y_size - 1; y >= 0; y--)
							  for(i = -1; i <= 1; i++)
								  for(j = -1; j <= 1; j++) 
									  if(x + i >= 0 && y + j >= 0 && x + i < lmap->config.x_size && y + j < lmap->config.y_size && (i != 0 || j != 0)) {
										  v = lmap->distance[x + i][y + j] + ((i * j != 0) ? 1.414 : 1);
										  if(v < lmap->distance[x][y]) {
											  lmap->distance[x][y] = v;
											  lmap->x_offset[x][y] = lmap->x_offset[x + i][y + j] + i;
											  lmap->y_offset[x][y] = lmap->y_offset[x + i][y + j] + j;
										  }
									  }
}

void create_likelihood_map(localize_map_p lmap, 
				  float **prob, float std)
{
  int x, y;
  double p, max;

  /* Compute the probability of each cell given the standard deviation,
     or "fuzziness" of the likelihood map */
  max = 0;
  for(x = 0; x < lmap->config.x_size; x++)
    for(y = 0; y < lmap->config.y_size; y++) {
      p = exp(-0.5 * square(lmap->distance[x][y] * 
				   lmap->config.resolution / std));
      if(p > max)
	max = p;
      prob[x][y] = p;
    }
  
  /* Correct the map so most likely reading has probability 1 */
  for(x = 0; x < lmap->config.x_size; x++)
    for(y = 0; y < lmap->config.y_size; y++) {
      prob[x][y] /= max;
      if(prob[x][y] < SMALL_PROB)
	prob[x][y] = SMALL_PROB;
      prob[x][y] = log(prob[x][y]);
    }
}

void create_stretched_likelihood_map(localize_map_p lmap, 
					   float **prob, float std, double min_likelihood)
{
  int x, y;
  double p, max;

  /* Compute the probability of each cell given the standard deviation,
     or "fuzziness" of the likelihood map */
  max = 0;
  for(x = 0; x < lmap->config.x_size; x++)
    for(y = 0; y < lmap->config.y_size; y++) {
      p = exp(-0.5 * square(lmap->distance[x][y] * 
				   lmap->config.resolution / std));
      if(p > max)
	max = p;
      prob[x][y] = p;
    }
  
  /* Correct the map so most likely reading has probability 1 */
  for(x = 0; x < lmap->config.x_size; x++)
    for(y = 0; y < lmap->config.y_size; y++)	{
      prob[x][y] /= max;
      prob[x][y] = log( min_likelihood + (1.0-min_likelihood)*prob[x][y]);
    }
}

void to_localize_map(map_p cmap, localize_map_p lmap,
			    localize_param_p param)
{
  int i;

  /* copy map parameters from carmen map */
  lmap->config = cmap->config;

  /* add raw map into likelihood map */
  lmap->map = *cmap;

  /* allocate distance map */
  lmap->complete_distance = (float *)calloc(lmap->config.x_size *
					    lmap->config.y_size,
					    sizeof(float));
  ////test_alloc(lmap->complete_distance);
  lmap->distance = (float **)calloc(lmap->config.x_size, sizeof(float *));
  ////test_alloc(lmap->distance);
  for(i = 0; i < lmap->config.x_size; i++)
    lmap->distance[i] = lmap->complete_distance + i * lmap->config.y_size;
  
  /* allocate prob map */
  lmap->complete_prob = (float *)calloc(lmap->config.x_size *
					lmap->config.y_size, sizeof(float));
  ////test_alloc(lmap->complete_prob);
  lmap->prob = (float **)calloc(lmap->config.x_size, sizeof(float *));
  ////test_alloc(lmap->prob);
  for(i = 0; i < lmap->config.x_size; i++)
    lmap->prob[i] = lmap->complete_prob + i * lmap->config.y_size;

  /* allocate prob map */
  lmap->complete_prob = (float *)calloc(lmap->config.x_size *
					lmap->config.y_size, sizeof(float));
  ////test_alloc(lmap->complete_prob);
  lmap->prob = (float **)calloc(lmap->config.x_size, sizeof(float *));
  ////test_alloc(lmap->prob);
  for(i = 0; i < lmap->config.x_size; i++)
    lmap->prob[i] = lmap->complete_prob + i * lmap->config.y_size;

  /* allocate gprob map */
  lmap->complete_gprob = (float *)calloc(lmap->config.x_size *
					 lmap->config.y_size, sizeof(float));
  ////test_alloc(lmap->complete_gprob);
  lmap->gprob = (float **)calloc(lmap->config.x_size, sizeof(float *));
  ////test_alloc(lmap->gprob);
  for(i = 0; i < lmap->config.x_size; i++)
    lmap->gprob[i] = lmap->complete_gprob + i * lmap->config.y_size;

  /* allocate x offset map */
  lmap->complete_x_offset = (short int *)calloc(lmap->config.x_size *
						lmap->config.y_size,
						sizeof(short int));
  ////test_alloc(lmap->complete_x_offset);
  lmap->x_offset = (short int **)calloc(lmap->config.x_size, 
					sizeof(short int *));
  ////test_alloc(lmap->x_offset);
  for(i = 0; i < lmap->config.x_size; i++)
    lmap->x_offset[i] = lmap->complete_x_offset + i * lmap->config.y_size;

  /* allocate y offset map */
  lmap->complete_y_offset = (short int *)calloc(lmap->config.x_size *
						lmap->config.y_size,
						sizeof(short int));
  ////test_alloc(lmap->complete_y_offset);
  lmap->y_offset = (short int **)calloc(lmap->config.x_size,
					sizeof(short int *));
  ////test_alloc(lmap->y_offset);
  for(i = 0; i < lmap->config.x_size; i++)
    lmap->y_offset[i] = lmap->complete_y_offset + i * lmap->config.y_size;

  create_distance_map(cmap, lmap, param);
/*   create_likelihood_map(lmap, lmap->prob, param->lmap_std); */
/*   create_likelihood_map(lmap, lmap->gprob, param->global_lmap_std); */

  create_stretched_likelihood_map(lmap, lmap->prob, param->lmap_std, param->tracking_beam_minlikelihood);
  create_stretched_likelihood_map(lmap, lmap->gprob, param->global_lmap_std, param->global_beam_minlikelihood);

}

/* Writes a carmen map out to a ppm file */

void localize_write_map_to_ppm(char *filename, 
				      localize_map_p map)
{
  FILE *fp;
  int i, j;
  double max;
  
  max = -1e6;
  for(i = 0; i < map->config.x_size; i++)
    for(j = 0; j < map->config.y_size; j++)
      if(map->map.map[i][j] > max)
	max = map->map.map[i][j];

  fp = fopen(filename, "w");
  fprintf(fp, "P6\n");
  fprintf(fp, "%d %d\n", map->config.x_size, map->config.y_size);
  fprintf(fp, "255\n");
  for(i = map->config.y_size - 1; i >= 0; i--)
    for(j = 0; j < map->config.x_size; j++)
      if(map->map.map[j][i] == -1) {
	fprintf(fp, "%c", 0);
	fprintf(fp, "%c", 0);
	fprintf(fp, "%c", 255);
      }
      else {
	fprintf(fp, "%c", 255 - (int)(map->map.map[j][i] / 
				      max * 255.0));
	fprintf(fp, "%c", 255 - (int)(map->map.map[j][i] / 
				      max * 255.0));
	fprintf(fp, "%c", 255 - (int)(map->map.map[j][i] / 
				      max * 255.0));
      }
  fclose(fp);
}

/* Writes a distance map out to a ppm file */

void localize_write_distance_map_to_ppm(char *filename, 
					       localize_map_p map)
{
  FILE *fp;
  int i, j;
  double max;

  max = -1e6;
  for(i = 0; i < map->config.x_size; i++)
    for(j = 0; j < map->config.y_size; j++)
      if(map->distance[i][j] > max)
	max = map->distance[i][j];

  fp = fopen(filename, "w");
  fprintf(fp, "P6\n");
  fprintf(fp, "%d %d\n", map->config.x_size, map->config.y_size);
  fprintf(fp, "255\n");
  for(i = map->config.y_size - 1; i >= 0; i--)
    for(j = 0; j < map->config.x_size; j++) {
      fprintf(fp, "%c", 255 - (int)(map->distance[j][i] / max * 255.0));
      fprintf(fp, "%c", 255 - (int)(map->distance[j][i] / max * 255.0));
      fprintf(fp, "%c", 255 - (int)(map->distance[j][i] / max * 255.0));
    }

  fclose(fp);
}

/* Writes a likelihood map out to a ppm file */

void localize_write_likelihood_map_to_ppm(char *filename, 
						 localize_map_p map,
						 float **prob)
{
  FILE *fp;
  int i, j;
  double p, max;
  
  max = -1e6;
  for(i = 0; i < map->config.x_size; i++)
    for(j = 0; j < map->config.y_size; j++)
      if(prob[i][j] > max)
	max = prob[i][j];
  max = exp(max);

  fp = fopen(filename, "w");
  fprintf(fp, "P6\n");
  fprintf(fp, "%d %d\n", map->config.x_size, map->config.y_size);
  fprintf(fp, "255\n");
  for(i = map->config.y_size - 1; i >= 0; i--)
    for(j = 0; j < map->config.x_size; j++) {
      p = exp(prob[j][i]);
      fprintf(fp, "%c", 255 - (int)(p / max * 255.0));
      fprintf(fp, "%c", 255 - (int)(p / max * 255.0));
      fprintf(fp, "%c", 255 - (int)(p / max * 255.0));
    }
  fclose(fp);
}






