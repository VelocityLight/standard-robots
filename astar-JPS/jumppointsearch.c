#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <cstdio>
#include <fstream>

#include "jps_grid.h"
#include "neighbors.h"
#include "path.h"
#include "display.h"
#include "heap.h"

using namespace std;

const int MAX_FILE_LENGTH = 120;
const int IMAGE_HEIGHT = 2100;
const int IMAGE_WIDTH = 2100;
const int type_length = 20;
/* Malloc count is used to make sure the program cleans up properly after itself. */
/* This value gets increased for every "malloc", and reduced for every "free". */

typedef struct IMAGE_DATA
{
	char c_file_path[MAX_FILE_LENGTH];
	unsigned char file_data[IMAGE_HEIGHT][IMAGE_WIDTH];
	int width, height, gray;
	char image_type[type_length];
}IMAGE_DATA;

int malloc_count = 0;

void ReadFile(struct IMAGE_DATA *image_pointer, char *file_path)
{
	int i,j;
	FILE *fp = fopen(file_path,"rb");
	char c_width[10], c_height[10], maxvalue[10];
	image_pointer->width = 0;
	image_pointer->height = 0;
	image_pointer->gray = 0;
	fscanf(fp, "%s %s %s %s", image_pointer->image_type, c_width, c_height, maxvalue);
	for(i = 0; c_width[i] != '\0'; i++){
		image_pointer->width = image_pointer->width * 10 + c_width[i] - '0';
	}
	for(i = 0; c_height[i] != '\0'; i++){
		image_pointer->height = image_pointer->height * 10 + c_height[i] - '0';
	}
	for(i = 0; maxvalue[i] != '\0'; i++){
		image_pointer->gray = image_pointer->gray * 10 + maxvalue[i] - '0';
	}
	strcpy(image_pointer->c_file_path, file_path);
	
	printf("%s %d %d %d\n",image_pointer->image_type, image_pointer->width, image_pointer->height, image_pointer->gray);
	unsigned char value;
	unsigned char useless;	
	useless = fgetc(fp);
	
	for (i = 0; i < image_pointer->height; ++ i)
	{
		for (j = 0; j < image_pointer->width; ++ j)
		{
			value = fgetc(fp);
			image_pointer->file_data[i][j] = value;
		}
	}
	fclose(fp);
}

void test(){
	struct IMAGE_DATA *image;
	char file_path[MAX_FILE_LENGTH] = "/home/jc/Desktop/test_map/small_map_cost.pgm";
	int i, startX = 0, startY = 0, endX = 0, endY = 0; /* Set the size of the map */
	int count;
	bool **matrix;
	struct grid newgrid;
	struct neighbor_xy_list *path_head = NULL, *path_pos = NULL;
	clock_t c0, c1;
	double runtime_diff_ms;

	image = new IMAGE_DATA;
	
	scanf("%d %d %d %d",&startX, &startY, &endX, &endY);
	
	ReadFile(image, file_path);
	if(startX == endX && startY == endY){
		printf("起点==终点！\n");
		return;
	}
	/* Prepare the Matrix of Walkable / Not Walkable - Dynamic Size */
	matrix = (bool **) malloc(image->height * sizeof(bool *));
	malloc_count++; /* [ Malloc Count ] */
	for (i = 0; i < image->height; i++) {
		matrix[i] = (bool *)malloc(image->width * sizeof(bool));
		malloc_count++; /* [ Malloc Count ] */
	}
	printf("Malloc count is:%d\n", malloc_count);
	unsigned char black = 0;
	unsigned char white = 255;
	for(int i=0; i<image->height; i++){
		for(int j=0; j<image->width; j++){
			if(i == startX && j == startY){
				printf("Start point found at X:%d / Y:%d\n", i, j);
				matrix[i][j] = true;

			}else if(i == endX && j == endY){
				printf("End point found at X:%d / Y:%d\n", i, j);
				matrix[i][j] = true;
			}else{
				if(black == image->file_data[i][j])
					matrix[i][j] = false;
				else if(white == image->file_data[i][j])
					matrix[i][j] = true;
			}
		}
	}

	newgrid = createGrid(image->width, image->height, matrix); /* Create a new grid */

	c0 = clock();

	path_head = findPath(&newgrid, startX, startY, endX, endY);
	path_pos = path_head;

	c1 = clock();
	runtime_diff_ms = (c1 - c0) * 1000. / CLOCKS_PER_SEC;

	printf("\nPath calculation took %.0fms\n", runtime_diff_ms);

	path_head = smooth_path(&newgrid, path_head);
	path_pos = path_head;

	printf("\n\n:Waypoints:\n");
	count = 0;
	
	unsigned char temp = 128;
	while (path_head != NULL && (path_head != (path_pos = path_pos->left))) {
		//printf("Step %d: x:%d y:%d\n", count++, path_pos->x, path_pos->y);
		image->file_data[path_pos->y][path_pos->x] = temp;
	}
	/*
	printf("::Path Smoothened::\n");
	
	printf("\n\n");
	if (path_head != NULL) {
		displaySolution(&newgrid, path_head);
	}
	*/

	ofstream cp("copy.pgm");
	cp << image->image_type << endl << image->width << " " << image->height << endl << image->gray<<endl;
	
	for (int i = 0; i < image->height; ++ i)
	{
		for (int j = 0; j < image->width; ++ j)
		{
			
			cp << image->file_data[i][j];
			//cp << temp;
		}
	}
	cp.close();

	/* Cleanup - Free all created arrays. */
	neighbor_xy_clean(path_head);

	for (i = 0; i < image->height; i++) {
		free(newgrid.nodes[i]);
		malloc_count--; /* [ Malloc Count ] */
	}

	free(newgrid.nodes);
	malloc_count--; /* [ Malloc Count ] */

	for (i = 0; i < image->height; i++) {
		free(matrix[i]);
		malloc_count--; /* [ Malloc Count ] */
	}
	free(matrix);
	malloc_count--; /* [ Malloc Count ] */

	printf("Malloc count is:%d\n", malloc_count);
	return;
}

int main(int argc, char **argv)
{
	test();
	return 0;
}
