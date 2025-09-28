#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>   // for time() used in srand()

// Struct to represent the grid with blocked/unblocked cells
typedef struct {
    int rows, cols;
    bool **blocked;  // 2D array: true = blocked, false = free
} Grid;

// Create a new grid of size rows x cols, marking blocked cells from the list
Grid *create_grid(int rows, int cols, int blocked_count, const int blocked_list[][2]) {
    Grid *g = (Grid*)malloc(sizeof(Grid));
    if (!g) {
        fprintf(stderr, "Memory allocation failed for Grid\n");
        exit(1);
    }
    g->rows = rows;
    g->cols = cols;
    // Allocate 2D array for blocked cells
    g->blocked = (bool**)malloc(rows * sizeof(bool*));
    if (!g->blocked) {
        fprintf(stderr, "Memory allocation failed for grid rows\n");
        free(g);
        exit(1);
    }
    for (int i = 0; i < rows; i++) {
        g->blocked[i] = (bool*)malloc(cols * sizeof(bool));
        if (!g->blocked[i]) {
            fprintf(stderr, "Memory allocation failed for grid row %d\n", i);
            // free previously allocated rows
            for (int r = 0; r < i; r++) free(g->blocked[r]);
            free(g->blocked);
            free(g);
            exit(1);
        }
        // Initialize all cells as unblocked (false)
        for (int j = 0; j < cols; j++) {
            g->blocked[i][j] = false;
        }
    }
    // Mark blocked cells from the list (assuming 0-based indices)
    for (int i = 0; i < blocked_count; i++) {
        int r = blocked_list[i][0];
        int c = blocked_list[i][1];
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            g->blocked[r][c] = true;
        }
    }
    return g;
}

// Free memory allocated for the grid
void free_grid(Grid *g) {
    if (!g) return;
    for (int i = 0; i < g->rows; i++) {
        free(g->blocked[i]);
    }
    free(g->blocked);
    free(g);
}

// Print grid: '.' free, '#' blocked
void print_grid(const Grid *g) {
    if (!g) return;
    for (int r = 0; r < g->rows; r++) {
        for (int c = 0; c < g->cols; c++) putchar(g->blocked[r][c] ? '#' : '.');
        putchar('\n');
    }
}

// Solve the path planning problem: find a path covering as many unique free cells as possible
// under the movement limit. Uses a greedy heuristic: always move to an unvisited neighbor if possible,
// otherwise move to a neighbor that leads towards unvisited cells.
void solve_path(Grid *g, int movement_points) {
    int rows = g->rows;
    int cols = g->cols;
    // Check for any unblocked start cell
    int start_r = -1, start_c = -1;
    for (int i = 0; i < rows && start_r < 0; i++) {
        for (int j = 0; j < cols; j++) {
            if (!g->blocked[i][j]) {
                start_r = i;
                start_c = j;
                break;
            }
        }
    }
    if (start_r < 0) {
        // No unblocked cell found
        printf("Unique squares visited: 0\n");
        return;
    }
    // Allocate visited array
    bool **visited = (bool**)malloc(rows * sizeof(bool*));
    for (int i = 0; i < rows; i++) {
        visited[i] = (bool*)malloc(cols * sizeof(bool));
        for (int j = 0; j < cols; j++) {
            visited[i][j] = false;
        }
    }
    // Arrays to store the path coordinates
    int max_path_len = movement_points + 1;
    int *path_r = (int*)malloc(max_path_len * sizeof(int));
    int *path_c = (int*)malloc(max_path_len * sizeof(int));

    // Starting position
    int cr = start_r, cc = start_c;
    visited[cr][cc] = true;
    path_r[0] = cr;
    path_c[0] = cc;
    int pathLen = 1;
    int unique_count = 1;

    // Define direction vectors (up, right, down, left)
    int dr[4] = {-1, 0, 1, 0};
    int dc[4] = {0, 1, 0, -1};

    // Attempt to move up to movement_points steps
    for (int step = 0; step < movement_points; step++) {
        bool moved = false;
        // First try to find an unvisited neighboring cell
        for (int i = 0; i < 4; i++) {
            int nr = cr + dr[i];
            int nc = cc + dc[i];
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                if (!g->blocked[nr][nc] && !visited[nr][nc]) {
                    // Move to this new cell
                    cr = nr;
                    cc = nc;
                    visited[cr][cc] = true;
                    path_r[pathLen] = cr;
                    path_c[pathLen] = cc;
                    pathLen++;
                    unique_count++;
                    moved = true;
                    break;
                }
            }
        }
        if (moved) continue;
        // No unvisited neighbor found; try a visited neighbor that has an unvisited neighbor
        for (int i = 0; i < 4 && !moved; i++) {
            int nr = cr + dr[i];
            int nc = cc + dc[i];
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                if (!g->blocked[nr][nc] && visited[nr][nc]) {
                    // Check neighbors of (nr, nc)
                    for (int j = 0; j < 4; j++) {
                        int r2 = nr + dr[j];
                        int c2 = nc + dc[j];
                        if (r2 >= 0 && r2 < rows && c2 >= 0 && c2 < cols) {
                            if (!g->blocked[r2][c2] && !visited[r2][c2]) {
                                // Move to the visited neighbor (backtrack step)
                                cr = nr;
                                cc = nc;
                                path_r[pathLen] = cr;
                                path_c[pathLen] = cc;
                                pathLen++;
                                moved = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
        if (!moved) {
            // No move possible that increases coverage; stop early
            break;
        }
    }

    // Print the path and count of unique visited cells
    printf("Path:");
    for (int i = 0; i < pathLen; i++) {
        printf(" (%d,%d)", path_r[i], path_c[i]);
    }
    printf("\nUnique squares visited: %d\n", unique_count);

    // Free allocated memory for visited and path arrays
    for (int i = 0; i < rows; i++) {
        free(visited[i]);
    }
    free(visited);
    free(path_r);
    free(path_c);
}

// Generate num_blocked unique random blocked cells inside the grid.
// If num_blocked > number of currently-free cells, it will block all free cells.
void generate_blocked(Grid *g, int num_blocked) {
    if (!g || num_blocked <= 0) return;

    // Count already blocked cells
    long already_blocked = 0;
    for (int r = 0; r < g->rows; r++) {
        for (int c = 0; c < g->cols; c++) {
            if (g->blocked[r][c]) already_blocked++;
        }
    }

    long total_cells = (long)g->rows * (long)g->cols;
    long available = total_cells - already_blocked;
    if (available <= 0) return;

    if (num_blocked > available) num_blocked = (int)available;

    int placed = 0;
    // Simple random sampling with rejection; OK unless blocking nearly whole grid.
    while (placed < num_blocked) {
        int r = rand() % g->rows;
        int c = rand() % g->cols;
        if (g->blocked[r][c]) continue; // already blocked, try again
        g->blocked[r][c] = true;
        placed++;
    }
}

// Main function with test cases
int main() {
    // Test 1: Tiny grid 1x1, no blocked cells
    {
        const int N = 1, M = 1;
        const int (*blocked_cells1)[2] = NULL;  // no blocked cells
        Grid *g1 = create_grid(N, M, 0, blocked_cells1);
        printf("Test 1 (%dx%d, no blocks):\n", N, M);
        solve_path(g1, 1);
        print_grid(g1);
        free_grid(g1);
        printf("\n");
    }
    // Test 2: Grid 2x2 where all cells are blocked
    {
        const int N = 2, M = 2;
        const int blocked_cells2[][2] = {{0,0}, {0,1}, {1,0}, {1,1}};
        Grid *g2 = create_grid(N, M, 4, blocked_cells2);
        printf("Test 2 (%dx%d, all blocked):\n", N, M);
        solve_path(g2, 10);
        print_grid(g2);
        free_grid(g2);
        printf("\n");
    }
    // Test 3: Grid with a single possible path (3x3 with a blocked center row)
    {
        const int N = 3, M = 3;
        const int blocked_cells3[][2] = {{1,0}, {1,1}, {1,2}};
        Grid *g3 = create_grid(N, M, 3, blocked_cells3);
        printf("Test 3 (%dx%d, one path):\n", N, M);
        solve_path(g3, 5);
        print_grid(g3);
        free_grid(g3);
        printf("\n");
    }
    // Test 4: Larger grid with many free cells (5x5 no blocks)
    {
        const int N = 5, M = 5;
        const int (*blocked_cells4)[2] = NULL;  // no blocked cells
        Grid *g4 = create_grid(N, M, 0, blocked_cells4);
        printf("Test 4 (%dx%d, no blocks):\n", N, M);
        solve_path(g4, 30);
        print_grid(g4);
        free_grid(g4);
        printf("\n");
    }
    
    srand((unsigned) time(NULL)); // Seed RNG once
    // Test 5: Randomly generated grid (10x10 with 20 random blocked cells)
    {
        const int N = 100, M = 10;
        Grid *g5 = create_grid(N, M, 0, NULL);
        generate_blocked(g5, 200);
        printf("Test 5 (%dx%d, random blocks):\n", N, M);
        solve_path(g5, 50);
        print_grid(g5);
        free_grid(g5);
        printf("\n");
    }
    return 0;
}
