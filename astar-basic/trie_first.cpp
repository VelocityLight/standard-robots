#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstring>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <queue>
#include <cmath>
#include <math.h>
#include <time.h>

#define UNIMPEDED 0
#define STARTNODE 1
#define ENDNODE 2
#define BARRIER 3

using namespace std;

const int MAX_FILE_LENGTH = 120;
const int IMAGE_HEIGHT = 5200;
const int IMAGE_WIDTH = 5200;
const int type_length = 10;

typedef struct IMAGE_DATA
{
	char c_file_path[MAX_FILE_LENGTH];					// 每个图像的文件路径
	unsigned char	file_data[IMAGE_HEIGHT][IMAGE_WIDTH];			// 每个图像的数据
	int width, height, gray;
	char image_type[type_length];
}IMAGE_DATA;

typedef struct AStarNode
{
	int s_x;	// 坐标(最终输出路径需要)
	int s_y;
	int s_g;	// 起点到此点的距离( 由g和h可以得到f，此处f省略，f=g+h )
	int s_h;	// 启发函数预测的此点到终点的距离
	int s_style;// 结点类型：起始点，终点，障碍物
	struct AStarNode * s_parent;	// 父节点
	int s_is_in_closetable;		// 是否在close表中
	int s_is_in_opentable;		// 是否在open表中
}AStarNode, *pAStarNode;

AStarNode  map_maze[IMAGE_HEIGHT][IMAGE_WIDTH];		// 结点数组
pAStarNode open_table[IMAGE_HEIGHT*IMAGE_WIDTH];		// open表
pAStarNode close_table[IMAGE_HEIGHT*IMAGE_WIDTH];		// close表
int open_node_count; //open表中节点数量
int close_node_count; //close表中结点数量
pAStarNode path_stack[IMAGE_HEIGHT*IMAGE_WIDTH];		// 保存路径的栈
int top = -1;			// 栈顶


// 交换两个元素
void swap( int idx1, int idx2 )
{
	pAStarNode tmp = open_table[idx1];
	open_table[idx1] = open_table[idx2];
	open_table[idx2] = tmp;
}

// 堆调整
void adjust_heap( int nIndex )
{
	int curr = nIndex;
	int child = curr * 2 + 1;	// 得到左孩子idx( 下标从0开始，所有做孩子是curr*2+1 )
	int parent = ( curr - 1 ) / 2;	// 得到双亲idx

	if (nIndex < 0 || nIndex >= open_node_count)
	{
		return;
	}

	// 往下调整( 要比较左右孩子和cuur parent )
	while ( child < open_node_count )
	{
		// 小根堆是双亲值小于孩子值
		if ( child + 1 < open_node_count && open_table[child]->s_g + open_table[child]->s_h  > open_table[child+1]->s_g + open_table[child+1]->s_h )
		{
			++child;// 判断左右孩子大小
		}

		if (open_table[curr]->s_g + open_table[curr]->s_h <= open_table[child]->s_g + open_table[child]->s_h)
		{
			break;
		}
		else
		{
			swap( child, curr );			// 交换节点
			curr = child;				// 再判断当前孩子节点
			child = curr * 2 + 1;			// 再判断左孩子
		}
	}

	if (curr != nIndex)
	{
		return;
	}

	// 往上调整( 只需要比较cuur child和parent )
	while (curr != 0)
	{
		if (open_table[curr]->s_g + open_table[curr]->s_h >= open_table[parent]->s_g + open_table[parent]->s_h)
		{
			break;
		}
		else
		{
			swap( curr, parent );
			curr = parent;
			parent = (curr-1)/2;
		}
	}
}

// 判断邻居点是否可以进入open表
void insert_to_opentable( int x, int y, pAStarNode curr_node, pAStarNode end_node, int w )
{
	int i;

	if ( map_maze[x][y].s_style != BARRIER )		// 不是障碍物
	{
		if ( !map_maze[x][y].s_is_in_closetable )	// 不在闭表中
		{
			if ( map_maze[x][y].s_is_in_opentable )	// 在open表中
			{
				// 需要判断是否是一条更优化的路径
				if ( map_maze[x][y].s_g > curr_node->s_g + w )	// 如果更优化
				{
					map_maze[x][y].s_g = curr_node->s_g + w;
					map_maze[x][y].s_parent = curr_node;

					for ( i = 0; i < open_node_count; ++i )
					{
						if ( open_table[i]->s_x == map_maze[x][y].s_x && open_table[i]->s_y == map_maze[x][y].s_y )
						{
							break;
						}
					}

					adjust_heap( i );					// 下面调整点
				}
			}
			else									// 不在open中
			{
				map_maze[x][y].s_g = curr_node->s_g + w;
				map_maze[x][y].s_h = abs(end_node->s_x - x ) + abs(end_node->s_y - y);
				map_maze[x][y].s_parent = curr_node;
				map_maze[x][y].s_is_in_opentable = 1;
				open_table[open_node_count++] = &(map_maze[x][y]);
			}
		}
	}
}

// 查找邻居
// 对上下左右8个邻居进行查找
void get_neighbors( pAStarNode curr_node, pAStarNode end_node )
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;

	// 下面对于8个邻居进行处理！
	if ( ( x + 1 ) >= 0 && ( x + 1 ) < IMAGE_HEIGHT && y >= 0 && y < IMAGE_WIDTH )
	{
		insert_to_opentable( x+1, y, curr_node, end_node, 10 );
	}

	if ( ( x - 1 ) >= 0 && ( x - 1 ) < IMAGE_HEIGHT && y >= 0 && y < IMAGE_WIDTH )
	{
		insert_to_opentable( x-1, y, curr_node, end_node, 10 );
	}

	if ( x >= 0 && x < IMAGE_HEIGHT && ( y + 1 ) >= 0 && ( y + 1 ) < IMAGE_WIDTH )
	{
		insert_to_opentable( x, y+1, curr_node, end_node, 10 );
	}

	if ( x >= 0 && x < IMAGE_HEIGHT && ( y - 1 ) >= 0 && ( y - 1 ) < IMAGE_WIDTH )
	{
		insert_to_opentable( x, y-1, curr_node, end_node, 10 );
	}

	if ( ( x + 1 ) >= 0 && ( x + 1 ) < IMAGE_HEIGHT && ( y + 1 ) >= 0 && ( y + 1 ) < IMAGE_WIDTH )
	{
		insert_to_opentable( x+1, y+1, curr_node, end_node, 14 );
	}

	if ( ( x + 1 ) >= 0 && ( x + 1 ) < IMAGE_HEIGHT && ( y - 1 ) >= 0 && ( y - 1 ) < IMAGE_WIDTH )
	{
		insert_to_opentable( x+1, y-1, curr_node, end_node, 14 );
	}

	if ( ( x - 1 ) >= 0 && ( x - 1 ) < IMAGE_HEIGHT && ( y + 1 ) >= 0 && ( y + 1 ) < IMAGE_WIDTH )
	{
		insert_to_opentable( x-1, y+1, curr_node, end_node, 14 );
	}

	if ( ( x - 1 ) >= 0 && ( x - 1 ) < IMAGE_HEIGHT && ( y - 1 ) >= 0 && ( y - 1 ) < IMAGE_WIDTH )
	{
		insert_to_opentable( x-1, y-1, curr_node, end_node, 14 );
	}
}


size_t ConvertHexStrToInt(const char* hex_str,size_t length)  
{  
	size_t sum = 0;  
	for(size_t i = 0;i < length; ++i)  
	{  
		int asc = (int)hex_str[i];  
		size_t r1 =(asc & 0x40)?(asc&0x0F)+0x9:(asc & 0x0F);  
		sum+=(r1*pow(16,length-i-1));  
	}  
	return sum;  
}

void ReadFile(struct IMAGE_DATA *image_pointer, char *file_path)
{
	int i,j;
	FILE *fp = fopen(file_path,"rb");
	char c_width[10], c_height[10], maxvalue[10];
	image_pointer->width = 0;
	image_pointer->height = 0;
	image_pointer->gray = 0;
	fscanf(fp, "%s %s %s %s", image_pointer->image_type, c_width, c_height, maxvalue);	// temp = "P5"
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
	
	//printf("%s %d %d %d\n",image_pointer->image_type, image_pointer->width, image_pointer->height, image_pointer->gray);
	unsigned char value;
	unsigned char useless;	
	useless = fgetc(fp);
	//ofstream fout("out.txt");
	//fout << useless << endl;
	for (i = 0; i < image_pointer->height; ++ i)
	{
		for (j = 0; j < image_pointer->width; ++ j)
		{
			value = fgetc(fp);
			image_pointer->file_data[i][j] = value;
			//fout << value << " ";
		}
		//fout << endl;
	}
	//fout.close();
	/*
	ofstream cp("copy.pgm");
	cp << image_pointer->image_type << endl << image_pointer->width << " " << image_pointer->height << endl << image_pointer->gray<<endl;
	unsigned char temp = 128;
	for (i = 0; i < image_pointer->height; ++ i)
	{
		for (j = 0; j < image_pointer->width; ++ j)
		{
			
			cp << image_pointer->file_data[i][j];
			//cp << temp;
		}
	}
	cp.close();
	*/
	fclose(fp);
}



int main(){
	clock_t start, end;
	
	struct IMAGE_DATA *image;
	image = new IMAGE_DATA;
	char file_path[MAX_FILE_LENGTH] = "/home/jc/Desktop/test_map/medium_map_cost.pgm";
	//printf("%s\n",file_path);
	AStarNode *start_node;			// 起始点
	AStarNode *end_node;			// 结束点
	AStarNode *curr_node;			// 当前点
	int is_found;			// 是否找到路径
	int start_x, start_y, end_x, end_y;
	/*
	char hex_str[MAX_FILE_LENGTH];
	while(scanf("%s",hex_str) != EOF){
		printf("%d\n", ConvertHexStrToInt(hex_str, strlen(hex_str)));
	}
	*/
	
	scanf("%d %d %d %d",&start_x, &start_y, &end_x, &end_y);
	
	ReadFile(image, file_path);
	if(start_x == end_x && start_y == end_y){
		printf("起点==终点！\n");
		return 0;
	}
	unsigned char black = 0;
	unsigned char white = 255;
	//cout << image->height << " " << image->width << endl;
	for(int i=0; i<image->height; i++){
		for(int j=0; j<image->width; j++){
			map_maze[i][j].s_g = 0;
			map_maze[i][j].s_h = 0;
			map_maze[i][j].s_is_in_closetable = 0;
			map_maze[i][j].s_is_in_opentable = 0;
			map_maze[i][j].s_style = image->file_data[i][j];
			map_maze[i][j].s_x = i;
			map_maze[i][j].s_y = j;
			map_maze[i][j].s_parent = NULL;
			
			if(i == start_x && j == start_y){
				map_maze[i][j].s_style = STARTNODE;
				start_node = &(map_maze[i][j]);
			}else if(i == end_x && j == end_y){
				map_maze[i][j].s_style = ENDNODE;
				end_node = &(map_maze[i][j]);
			}else{
				if(black == image->file_data[i][j])
					map_maze[i][j].s_style = BARRIER;
				else if(white == image->file_data[i][j])
					map_maze[i][j].s_style = UNIMPEDED;
			}
			//printf("%d ", image->file_data[i][j]);
		}
		//printf("\n");
	}
	// 下面使用A*算法得到路径
	start  = clock();
	open_table[open_node_count++] = start_node;			// 起始点加入open表

	start_node->s_is_in_opentable = 1;				// 加入open表
	start_node->s_g = 0;
	start_node->s_h = abs(end_node->s_x - start_node->s_x) + abs(end_node->s_y - start_node->s_y);
	start_node->s_parent = NULL;
	is_found = 0;
	int cnt = 1;
	while( 1 )
	{
		// for test
		/*
		for ( x = 0; x < open_node_count; ++x )
		{
			printf("(%d,%d):%d   ", open_table[x]->s_x, open_table[x]->s_y, open_table[x]->s_g+open_table[x]->s_h);
		}
		printf("\n\n");
		*/
		curr_node = open_table[0];		// open表的第一个点一定是f值最小的点(通过堆排序得到的)
		open_table[0] = open_table[--open_node_count];	// 最后一个点放到第一个点，然后进行堆调整
		adjust_heap( 0 );				// 调整堆

		close_table[close_node_count++] = curr_node;	// 当前点加入close表
		curr_node->s_is_in_closetable = 1;		// 已经在close表中了

		if ( curr_node->s_x == end_node->s_x && curr_node->s_y == end_node->s_y )// 终点在close中，结束
		{
			is_found = 1;
			break;
		}

		get_neighbors( curr_node, end_node );			// 对邻居的处理

		if ( open_node_count == 0 )				// 没有路径到达
		{
			is_found = 0;
			break;
		}
		//printf("cnt=%d\n",cnt++);
		cnt++;
	}

	if ( is_found )
	{
		curr_node = end_node;

		while( curr_node )
		{
			path_stack[++top] = curr_node;
			curr_node = curr_node->s_parent;

		}
		while( top >= 0 )		//输出路径
		{

			if ( top > 0 )
			{
				//printf("(%d,%d)-->", path_stack[top]->s_x, path_stack[top]->s_y);
				image->file_data[path_stack[top]->s_x][path_stack[top]->s_y] = black;
				top--;
			}
			else
			{
				//printf("(%d,%d)", path_stack[top]->s_x, path_stack[top]->s_y);
				image->file_data[path_stack[top]->s_x][path_stack[top]->s_y] = black;
				top--;
			}
		}
		
		ofstream cp("copy.pgm");
		cp << image->image_type << endl << image->width << " " << image->height << endl << image->gray<<endl;
		unsigned char temp = 128;
		for (int i = 0; i < image->height; ++ i)
		{
			for (int j = 0; j < image->width; ++ j)
			{
				
				cp << image->file_data[i][j];
				//cp << temp;
			}
		}
		cp.close();
		
	}
	else
	{
		printf("无路径可达");
	}
	printf("\n");
	end = clock();
	double list;
	list = (double)(end-start) / CLOCKS_PER_SEC;
	printf("%f seconds/n\npoint_number=%d\n",list,cnt);
	return 0;
}