use crossterm::{
    event::{self, Event, KeyCode},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use rand::Rng;
use ratatui::{
    Terminal,
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io;
use std::time::{Duration, Instant};
use std::{fs, path::PathBuf};

// Saves to ~/.ant-farm.json on Unix, %USERPROFILE%\.ant-farm.json on Windows
const SAVE_FILE: &str = ".ant-farm.json";
const TICK_RATE: Duration = Duration::from_millis(150); // Slower, smoother
const MAX_ANTS: usize = 30; // Reduced population cap
const PHEROMONE_DECAY: f32 = 0.92;
const SPAWN_RATE: f32 = 0.012; // Slightly faster spawning
const FOOD_SPAWN_RATE: f32 = 0.015;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
enum CellType {
    Solid,
    Tunnel,
    Food,
    Queen,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
enum AntState {
    Wandering,
    Foraging,
    Carrying,
    Digging,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Ant {
    x: usize,
    y: usize,
    state: AntState,
    target_x: Option<usize>,
    target_y: Option<usize>,
    idle_ticks: u32,
    dig_cooldown: u32,        // Cooldown before ant can dig again
    stuck_ticks: u32,         // Counter for detecting stuck ants
    last_dig_dx: Option<i32>, // Last digging direction X component
    last_dig_dy: Option<i32>, // Last digging direction Y component
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Pheromone {
    home: f32,
    distress: f32, // For low-health rescue signals
}

#[derive(Serialize, Deserialize)]
struct SerializableWorld {
    width: usize,
    height: usize,
    grid: Vec<Vec<CellType>>,
    ants: Vec<Ant>,
    pheromones: std::collections::HashMap<String, Pheromone>,
    queen_pos: (usize, usize),
    food_collected: u32,
    ticks: u64,
    occupancy: std::collections::HashMap<String, usize>,
    health: f32,
}

impl From<&World> for SerializableWorld {
    fn from(world: &World) -> Self {
        let mut pheromones = std::collections::HashMap::new();
        for ((x, y), pheromone) in &world.pheromones {
            pheromones.insert(format!("{},{}", x, y), pheromone.clone());
        }

        let mut occupancy = std::collections::HashMap::new();
        for ((x, y), count) in &world.occupancy {
            occupancy.insert(format!("{},{}", x, y), *count);
        }

        SerializableWorld {
            width: world.width,
            height: world.height,
            grid: world.grid.clone(),
            ants: world.ants.clone(),
            pheromones,
            queen_pos: world.queen_pos,
            food_collected: world.food_collected,
            ticks: world.ticks,
            occupancy,
            health: world.health,
        }
    }
}

impl From<SerializableWorld> for World {
    fn from(sw: SerializableWorld) -> Self {
        let mut pheromones = HashMap::new();
        for (key, pheromone) in sw.pheromones {
            if let Some((x, y)) = parse_coordinate_key(&key) {
                pheromones.insert((x, y), pheromone);
            }
        }

        let mut occupancy = HashMap::new();
        for (key, count) in sw.occupancy {
            if let Some((x, y)) = parse_coordinate_key(&key) {
                occupancy.insert((x, y), count);
            }
        }

        World {
            width: sw.width,
            height: sw.height,
            grid: sw.grid,
            ants: sw.ants,
            pheromones,
            queen_pos: sw.queen_pos,
            food_collected: sw.food_collected,
            ticks: sw.ticks,
            occupancy,
            health: sw.health,
        }
    }
}

fn parse_coordinate_key(key: &str) -> Option<(usize, usize)> {
    let parts: Vec<&str> = key.split(',').collect();
    if parts.len() == 2
        && let (Ok(x), Ok(y)) = (parts[0].parse::<usize>(), parts[1].parse::<usize>())
    {
        return Some((x, y));
    }
    None
}

struct World {
    width: usize,
    height: usize,
    grid: Vec<Vec<CellType>>,
    ants: Vec<Ant>,
    pheromones: HashMap<(usize, usize), Pheromone>,
    queen_pos: (usize, usize),
    food_collected: u32,
    ticks: u64,
    occupancy: HashMap<(usize, usize), usize>, // Track ants per cell to prevent overlap
    health: f32,                               // Colony health (0.0-100.0)
}

impl World {
    fn new(width: usize, height: usize) -> Self {
        let mut grid = vec![vec![CellType::Solid; width]; height];
        let queen_x = width / 2;
        let queen_y = height / 2;

        // Create initial chamber
        for dy in -3..=3 {
            for dx in -4..=4 {
                let x = queen_x as i32 + dx;
                let y = queen_y as i32 + dy;
                if x >= 0 && x < width as i32 && y >= 0 && y < height as i32 {
                    grid[y as usize][x as usize] = CellType::Tunnel;
                }
            }
        }
        grid[queen_y][queen_x] = CellType::Queen;

        // Initial ants
        let mut ants = Vec::new();
        for _ in 0..3 {
            // Start with fewer ants
            ants.push(Ant {
                x: queen_x,
                y: queen_y,
                state: AntState::Wandering,
                target_x: None,
                target_y: None,
                idle_ticks: 0,
                dig_cooldown: 0,
                stuck_ticks: 0,
                last_dig_dx: None,
                last_dig_dy: None,
            });
        }

        // Scatter some food around
        let mut rng = rand::rng();
        for _ in 0..15 {
            let x = rng.random_range(0..width);
            let y = rng.random_range(0..height);
            if grid[y][x] == CellType::Solid {
                grid[y][x] = CellType::Food;
            }
        }

        World {
            width,
            height,
            grid,
            ants,
            pheromones: HashMap::new(),
            queen_pos: (queen_x, queen_y),
            food_collected: 0,
            ticks: 0,
            occupancy: HashMap::new(),
            health: 100.0, // Start at full health
        }
    }

    fn save(&self) -> io::Result<()> {
        let home = std::env::var("HOME")
            .or_else(|_| std::env::var("USERPROFILE"))
            .unwrap_or_else(|_| ".".to_string());
        let path = PathBuf::from(home).join(SAVE_FILE);
        let serializable_world = SerializableWorld::from(self);
        let json = serde_json::to_string(&serializable_world)?;
        fs::write(path, json)?;
        Ok(())
    }

    fn load() -> io::Result<Self> {
        let home = std::env::var("HOME")
            .or_else(|_| std::env::var("USERPROFILE"))
            .unwrap_or_else(|_| ".".to_string());
        let path = PathBuf::from(home).join(SAVE_FILE);
        let json = fs::read_to_string(path)?;
        let serializable_world: SerializableWorld = serde_json::from_str(&json)?;
        Ok(World::from(serializable_world))
    }

    fn update(&mut self) {
        self.ticks += 1;
        let mut rng = rand::rng();

        // Health decay
        const HEALTH_DECAY: f32 = 0.07;
        self.health = (self.health - HEALTH_DECAY).max(0.0);

        // Check if colony is in low health state
        let low_health = self.health < 55.0;

        // Update occupancy tracking
        self.update_occupancy();

        // Decay pheromones with different rates based on health status
        if low_health {
            // Decay distress slower during crisis to build signals
            self.pheromones.retain(|_, p| {
                p.home *= PHEROMONE_DECAY;
                p.distress *= 0.95; // Slower decay for distress
                p.home > 0.1 || p.distress > 0.1
            });
        } else {
            // Normal: Decay distress faster when healthy
            self.pheromones.retain(|_, p| {
                p.home *= PHEROMONE_DECAY;
                p.distress *= 0.85; // Faster fade-out
                p.home > 0.1 || p.distress > 0.1
            });
        }

        // Decrease dig cooldowns
        for ant in &mut self.ants {
            if ant.dig_cooldown > 0 {
                ant.dig_cooldown -= 1;
            }
        }

        // Decide what each ant will do
        let mut actions = Vec::new();
        for i in 0..self.ants.len() {
            let ant = &self.ants[i];

            // Check if ant is near distress pheromones
            let near_distress = self
                .pheromones
                .get(&(ant.x, ant.y))
                .is_some_and(|p| p.distress > 0.5);

            let action = match ant.state {
                AntState::Wandering => {
                    if rng.random_bool(0.3) {
                        AntAction::ChangeState(AntState::Foraging)
                    } else if rng.random_bool(0.35) && self.has_nearby_diggable_area(ant.x, ant.y) {
                        AntAction::ChangeState(AntState::Digging)
                    } else if near_distress && rng.random_bool(0.8) {
                        // If near distress, heavily favor digging to help
                        AntAction::ChangeState(AntState::Digging)
                    } else {
                        AntAction::MoveRandom
                    }
                }
                AntState::Foraging => {
                    // Foraging ants should prioritize food over digging
                    if rng.random_bool(0.4) {
                        AntAction::ChangeState(AntState::Wandering)
                    } else if rng.random_bool(0.1) && self.has_nearby_diggable_area(ant.x, ant.y) {
                        AntAction::ChangeState(AntState::Digging)
                    } else if near_distress && rng.random_bool(0.6) {
                        // If near distress while foraging, switch to digging to help
                        AntAction::ChangeState(AntState::Digging)
                    } else {
                        AntAction::MoveRandomOrDig
                    }
                }
                AntState::Carrying => AntAction::MoveToTarget,
                AntState::Digging => {
                    // Digging ants should switch states more often
                    if rng.random_bool(0.3) {
                        AntAction::ChangeState(AntState::Wandering)
                    } else if rng.random_bool(0.2) {
                        AntAction::ChangeState(AntState::Foraging)
                    } else {
                        AntAction::TryDig
                    }
                }
            };
            actions.push((i, action));
        }

        // Execute actions
        for (i, action) in actions {
            match action {
                AntAction::ChangeState(state) => {
                    self.ants[i].state = state;
                    self.ants[i].idle_ticks = 0;
                    // Clear dig direction when changing states
                    self.ants[i].last_dig_dx = None;
                    self.ants[i].last_dig_dy = None;
                }
                AntAction::MoveRandom => {
                    // Check for nearby food first
                    let ant = &self.ants[i];
                    if let Some((food_x, food_y)) = self.has_nearby_food(ant.x, ant.y) {
                        // Move toward food
                        let dx = (food_x as i32 - ant.x as i32).signum();
                        let dy = (food_y as i32 - ant.y as i32).signum();
                        let new_x = (ant.x as i32 + dx) as usize;
                        let new_y = (ant.y as i32 + dy) as usize;

                        if new_x < self.width
                            && new_y < self.height
                            && self.grid[new_y][new_x] != CellType::Solid
                        {
                            self.ants[i].x = new_x;
                            self.ants[i].y = new_y;
                            self.ants[i].idle_ticks = 0;

                            // Check if we reached the food
                            if self.grid[new_y][new_x] == CellType::Food {
                                self.grid[new_y][new_x] = CellType::Tunnel;
                                self.ants[i].state = AntState::Carrying;
                                self.ants[i].target_x = Some(self.queen_pos.0);
                                self.ants[i].target_y = Some(self.queen_pos.1);
                            }
                        } else {
                            // Can't move toward food, try random movement
                            let (ant_x, ant_y) = (ant.x, ant.y);
                            let moved = self.move_ant_random(i, low_health);
                            if !moved {
                                self.ants[i].idle_ticks += 1;
                                if self.ants[i].idle_ticks > 2
                                    && self.has_nearby_diggable_area(ant_x, ant_y)
                                {
                                    self.ants[i].state = AntState::Digging;
                                    self.ants[i].idle_ticks = 0;
                                }
                            }
                        }
                    } else {
                        // No nearby food, do normal random movement
                        let (ant_x, ant_y) = (ant.x, ant.y);
                        let moved = self.move_ant_random(i, low_health);
                        if moved {
                            self.ants[i].idle_ticks = 0;
                            // Check for food at current position
                            let ant = &self.ants[i];
                            if self.grid[ant.y][ant.x] == CellType::Food {
                                self.grid[ant.y][ant.x] = CellType::Tunnel;
                                self.ants[i].state = AntState::Carrying;
                                self.ants[i].target_x = Some(self.queen_pos.0);
                                self.ants[i].target_y = Some(self.queen_pos.1);
                            }
                        } else {
                            self.ants[i].idle_ticks += 1;
                            if self.ants[i].idle_ticks > 2
                                && self.has_nearby_diggable_area(ant_x, ant_y)
                            {
                                self.ants[i].state = AntState::Digging;
                                self.ants[i].idle_ticks = 0;
                            }
                        }
                    }
                }
                AntAction::MoveRandomOrDig => {
                    // Foraging ants should prioritize food over digging
                    let ant = &self.ants[i];
                    if let Some((food_x, food_y)) = self.has_nearby_food(ant.x, ant.y) {
                        // Move toward food
                        let dx = (food_x as i32 - ant.x as i32).signum();
                        let dy = (food_y as i32 - ant.y as i32).signum();
                        let new_x = (ant.x as i32 + dx) as usize;
                        let new_y = (ant.y as i32 + dy) as usize;

                        if new_x < self.width
                            && new_y < self.height
                            && self.grid[new_y][new_x] != CellType::Solid
                        {
                            self.ants[i].x = new_x;
                            self.ants[i].y = new_y;
                            self.ants[i].idle_ticks = 0;

                            // Check if we reached the food
                            if self.grid[new_y][new_x] == CellType::Food {
                                self.grid[new_y][new_x] = CellType::Tunnel;
                                self.ants[i].state = AntState::Carrying;
                                self.ants[i].target_x = Some(self.queen_pos.0);
                                self.ants[i].target_y = Some(self.queen_pos.1);
                            }
                        } else {
                            // Can't move toward food, try random movement
                            let (ant_x, ant_y) = (ant.x, ant.y);
                            let moved = self.move_ant_random(i, low_health);
                            if !moved {
                                if self.has_nearby_diggable_area(ant_x, ant_y) {
                                    self.ants[i].state = AntState::Digging;
                                    self.ants[i].idle_ticks = 0;
                                }
                            } else {
                                self.ants[i].idle_ticks = 0;
                                // Check for food at current position
                                let ant = &self.ants[i];
                                if self.grid[ant.y][ant.x] == CellType::Food {
                                    self.grid[ant.y][ant.x] = CellType::Tunnel;
                                    self.ants[i].state = AntState::Carrying;
                                    self.ants[i].target_x = Some(self.queen_pos.0);
                                    self.ants[i].target_y = Some(self.queen_pos.1);
                                }
                            }
                        }
                    } else {
                        // No nearby food, do normal movement
                        let (ant_x, ant_y) = (ant.x, ant.y);
                        let moved = self.move_ant_random(i, low_health);
                        if !moved {
                            if self.has_nearby_diggable_area(ant_x, ant_y) {
                                self.ants[i].state = AntState::Digging;
                                self.ants[i].idle_ticks = 0;
                            }
                        } else {
                            self.ants[i].idle_ticks = 0;
                            // Check for food
                            let ant = &self.ants[i];
                            if self.grid[ant.y][ant.x] == CellType::Food {
                                self.grid[ant.y][ant.x] = CellType::Tunnel;
                                self.ants[i].state = AntState::Carrying;
                                self.ants[i].target_x = Some(self.queen_pos.0);
                                self.ants[i].target_y = Some(self.queen_pos.1);
                            }
                        }
                    }
                }
                AntAction::MoveToTarget => {
                    self.move_ant_toward_target(i);
                    let ant = &self.ants[i];

                    // Drop stronger pheromone trail for carrying ants
                    let pos = (ant.x, ant.y);
                    self.pheromones
                        .entry(pos)
                        .or_insert(Pheromone {
                            home: 0.0,
                            distress: 0.0,
                        })
                        .home += 2.0; // Stronger trail for food carriers

                    // Check if reached queen
                    if ant.x == self.queen_pos.0 && ant.y == self.queen_pos.1 {
                        self.food_collected += 1;
                        // Restore health
                        const HEALTH_RESTORE: f32 = 15.0;
                        self.health = (self.health + HEALTH_RESTORE).min(100.0);
                        self.ants[i].state = AntState::Wandering;
                        self.ants[i].target_x = None;
                        self.ants[i].target_y = None;
                    }
                    self.ants[i].idle_ticks = 0;
                }
                AntAction::TryDig => {
                    // Only try to dig if cooldown is finished
                    if self.ants[i].dig_cooldown == 0 && self.try_dig(i) {
                        self.ants[i].idle_ticks = 0;
                        self.ants[i].dig_cooldown = 2; // Shorter cooldown for faster tunneling
                        // More persistent digging - stay digging longer to form tunnels
                        if rng.random_bool(0.85) {
                            self.ants[i].state = AntState::Digging;
                        } else {
                            self.ants[i].state = AntState::Wandering;
                        }
                    } else {
                        // Couldn't dig (cooldown or no valid spots), try wandering
                        self.ants[i].idle_ticks += 1;
                        if self.ants[i].idle_ticks > 3 {
                            self.ants[i].state = AntState::Wandering;
                            self.ants[i].idle_ticks = 0;
                        }
                    }
                }
            }
        }

        // Distress signaling if low health
        if low_health {
            for i in 0..self.ants.len() {
                let ant = &self.ants[i];
                let is_carrying_or_near_food = matches!(ant.state, AntState::Carrying)
                    || self.has_nearby_food(ant.x, ant.y).is_some();
                if is_carrying_or_near_food {
                    // Deposit strong distress signal
                    let pos = (ant.x, ant.y);
                    self.pheromones
                        .entry(pos)
                        .or_insert(Pheromone {
                            home: 0.0,
                            distress: 0.0,
                        })
                        .distress += 3.0; // High strength to attract diggers
                }
            }
        }

        // Spawn new ants occasionally
        if self.ants.len() < MAX_ANTS && rng.random_bool(SPAWN_RATE as f64) {
            // Spawn ants in a small area around the queen, not all at the exact same spot
            let spawn_x = self.queen_pos.0 as i32 + rng.random_range(-2..=2);
            let spawn_y = self.queen_pos.1 as i32 + rng.random_range(-2..=2);
            let spawn_x = spawn_x.clamp(0, self.width as i32 - 1) as usize;
            let spawn_y = spawn_y.clamp(0, self.height as i32 - 1) as usize;

            // Only spawn if the position is a tunnel (not solid)
            if self.grid[spawn_y][spawn_x] == CellType::Tunnel {
                self.ants.push(Ant {
                    x: spawn_x,
                    y: spawn_y,
                    state: AntState::Digging, // Start new ants digging!
                    target_x: None,
                    target_y: None,
                    idle_ticks: 0,
                    dig_cooldown: 0,
                    stuck_ticks: 0,
                    last_dig_dx: None,
                    last_dig_dy: None,
                });
            }
        }

        // Spawn food
        if rng.random_bool(FOOD_SPAWN_RATE as f64) {
            let x = rng.random_range(0..self.width);
            let y = rng.random_range(0..self.height);
            if self.grid[y][x] == CellType::Solid {
                self.grid[y][x] = CellType::Food;
            }
        }
    }

    fn has_nearby_food(&self, x: usize, y: usize) -> Option<(usize, usize)> {
        // Check for food within 2 cells
        for dy in -2..=2 {
            for dx in -2..=2 {
                let check_x = x as i32 + dx;
                let check_y = y as i32 + dy;
                if check_x >= 0
                    && check_x < self.width as i32
                    && check_y >= 0
                    && check_y < self.height as i32
                    && self.grid[check_y as usize][check_x as usize] == CellType::Food
                {
                    return Some((check_x as usize, check_y as usize));
                }
            }
        }
        None
    }

    fn find_strongest_pheromone_neighbor(
        &self,
        x: usize,
        y: usize,
        low_health: bool,
    ) -> Option<(i32, i32)> {
        let mut strongest_dir = None;
        let mut strongest_strength = 0.0;

        // Check 8 neighboring cells (cardinal + diagonal)
        let directions = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ];

        for &(dx, dy) in &directions {
            let check_x = x as i32 + dx;
            let check_y = y as i32 + dy;

            if check_x >= 0
                && check_x < self.width as i32
                && check_y >= 0
                && check_y < self.height as i32
            {
                let pos = (check_x as usize, check_y as usize);
                if let Some(pheromone) = self.pheromones.get(&pos) {
                    // Prioritize distress pheromones when health is low
                    if low_health
                        && pheromone.distress > strongest_strength
                        && pheromone.distress > 0.5
                    {
                        strongest_strength = pheromone.distress;
                        strongest_dir = Some((dx, dy));
                    } else if !low_health
                        && pheromone.home > strongest_strength
                        && pheromone.home > 0.5
                    {
                        strongest_strength = pheromone.home;
                        strongest_dir = Some((dx, dy));
                    }
                }
            }
        }

        strongest_dir
    }

    fn can_move_to(&self, x: usize, y: usize) -> bool {
        // Check if cell is not solid and not overcrowded (max 2 ants per cell)
        self.grid[y][x] != CellType::Solid
            && self.occupancy.get(&(x, y)).map_or(0, |&count| count) < 2
    }

    fn update_occupancy(&mut self) {
        // Clear occupancy map
        self.occupancy.clear();

        // Count ants in each cell
        for ant in &self.ants {
            let pos = (ant.x, ant.y);
            *self.occupancy.entry(pos).or_insert(0) += 1;
        }
    }

    fn has_nearby_diggable_area(&self, x: usize, y: usize) -> bool {
        // Check if there are solid cells within 2 cells that can be dug
        for dy in -2..=2 {
            for dx in -2..=2 {
                let check_x = x as i32 + dx;
                let check_y = y as i32 + dy;
                if check_x >= 0
                    && check_x < self.width as i32
                    && check_y >= 0
                    && check_y < self.height as i32
                {
                    let check_x = check_x as usize;
                    let check_y = check_y as usize;
                    if self.grid[check_y][check_x] == CellType::Solid {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn move_ant_random(&mut self, idx: usize, low_health: bool) -> bool {
        let mut rng = rand::rng();
        let ant = &self.ants[idx];

        // Strongly bias movement away from queen to encourage exploration
        let dx_from_queen = ant.x as i32 - self.queen_pos.0 as i32;
        let dy_from_queen = ant.y as i32 - self.queen_pos.1 as i32;

        let mut directions = Vec::new();

        // Always prefer moving away from queen, with stronger bias when close
        let distance_from_queen = dx_from_queen.abs() + dy_from_queen.abs();

        if distance_from_queen <= 3 {
            // Very close to queen - strongly prefer moving away
            if dx_from_queen <= 0 {
                directions.push((-1, 0));
                directions.push((-1, 0));
                directions.push((-1, 0)); // Triple weight for moving away
            }
            if dx_from_queen >= 0 {
                directions.push((1, 0));
                directions.push((1, 0));
                directions.push((1, 0));
            }
            if dy_from_queen <= 0 {
                directions.push((0, -1));
                directions.push((0, -1));
                directions.push((0, -1));
            }
            if dy_from_queen >= 0 {
                directions.push((0, 1));
                directions.push((0, 1));
                directions.push((0, 1));
            }
        } else if distance_from_queen <= 6 {
            // Moderately close - still prefer moving away
            if dx_from_queen <= 0 {
                directions.push((-1, 0));
                directions.push((-1, 0));
            }
            if dx_from_queen >= 0 {
                directions.push((1, 0));
                directions.push((1, 0));
            }
            if dy_from_queen <= 0 {
                directions.push((0, -1));
                directions.push((0, -1));
            }
            if dy_from_queen >= 0 {
                directions.push((0, 1));
                directions.push((0, 1));
            }
            // Add some random movement too
            directions.extend_from_slice(&[(-1, 0), (1, 0), (0, -1), (0, 1)]);
        } else {
            // Far from queen - normal random movement
            directions.extend_from_slice(&[(-1, 0), (1, 0), (0, -1), (0, 1)]);
        }

        // Check for pheromone trails and bias movement toward them
        if let Some(pheromone_dir) =
            self.find_strongest_pheromone_neighbor(ant.x, ant.y, low_health)
        {
            // Add the pheromone direction 2-3 times to bias the movement
            directions.push(pheromone_dir);
            directions.push(pheromone_dir);
            directions.push(pheromone_dir);
        }

        // If low health, bias toward distress pheromones (additional check)
        if low_health {
            let mut best_dir = None;
            let mut best_distress = 0.0;

            // Check all 8 directions for distress pheromones
            let all_directions = [
                (-1, -1),
                (-1, 0),
                (-1, 1),
                (0, -1),
                (0, 1),
                (1, -1),
                (1, 0),
                (1, 1),
            ];

            for &dir in &all_directions {
                let nx = ant.x as i32 + dir.0;
                let ny = ant.y as i32 + dir.1;
                if nx >= 0
                    && nx < self.width as i32
                    && ny >= 0
                    && ny < self.height as i32
                    && let Some(p) = self.pheromones.get(&(nx as usize, ny as usize))
                    && p.distress > best_distress
                    && p.distress > 0.5
                // Lower threshold for more responsiveness
                {
                    best_distress = p.distress;
                    best_dir = Some(dir);
                }
            }

            if let Some(dir) = best_dir {
                // Instead of just moving toward distress, encourage digging toward it
                // Add the direction but also bias toward digging in that direction
                for _ in 0..3 {
                    directions.push(dir);
                }

                // If the distress direction leads to solid ground, heavily bias digging
                let distress_x = ant.x as i32 + dir.0;
                let distress_y = ant.y as i32 + dir.1;
                if distress_x >= 0
                    && distress_x < self.width as i32
                    && distress_y >= 0
                    && distress_y < self.height as i32
                {
                    let distress_x = distress_x as usize;
                    let distress_y = distress_y as usize;
                    if self.grid[distress_y][distress_x] == CellType::Solid {
                        // This direction leads to solid ground - encourage digging!
                        for _ in 0..8 {
                            // Very heavy bias for digging toward distress
                            directions.push(dir);
                        }
                    }
                }
            }
        }

        // Shuffle
        for i in (1..directions.len()).rev() {
            let j = rng.random_range(0..=i);
            directions.swap(i, j);
        }

        // Separate tunnel and non-tunnel directions for preference
        let mut tunnel_dirs = Vec::new();
        let mut other_dirs = Vec::new();

        for &dir in &directions {
            let new_x = ant.x as i32 + dir.0;
            let new_y = ant.y as i32 + dir.1;

            if new_x >= 0 && new_x < self.width as i32 && new_y >= 0 && new_y < self.height as i32 {
                let new_x = new_x as usize;
                let new_y = new_y as usize;
                if self.can_move_to(new_x, new_y) {
                    if self.grid[new_y][new_x] == CellType::Tunnel {
                        tunnel_dirs.push(dir);
                    } else {
                        other_dirs.push(dir);
                    }
                }
            }
        }

        // Prefer tunnels, but fall back to other spaces if needed
        let preferred_dirs = if !tunnel_dirs.is_empty() {
            &tunnel_dirs
        } else {
            &other_dirs
        };

        if let Some(&dir) = preferred_dirs.first() {
            let new_x = ant.x as i32 + dir.0;
            let new_y = ant.y as i32 + dir.1;
            let new_x = new_x as usize;
            let new_y = new_y as usize;
            self.ants[idx].x = new_x;
            self.ants[idx].y = new_y;
            return true;
        }
        false
    }

    fn move_ant_toward_target(&mut self, idx: usize) {
        let ant = &self.ants[idx];
        if let (Some(tx), Some(ty)) = (ant.target_x, ant.target_y) {
            let dx = (tx as i32 - ant.x as i32).signum();
            let dy = (ty as i32 - ant.y as i32).signum();

            // Try direct path first
            let new_x = (ant.x as i32 + dx) as usize;
            let new_y = (ant.y as i32 + dy) as usize;

            if new_x < self.width
                && new_y < self.height
                && self.grid[new_y][new_x] != CellType::Solid
            {
                self.ants[idx].x = new_x;
                self.ants[idx].y = new_y;
            } else {
                // If direct path blocked, try simple tunnel-aware movement
                self.move_ant_toward_target_simple(idx);
            }
        }
    }

    fn move_ant_toward_target_simple(&mut self, idx: usize) {
        let (ant_x, ant_y, target_x, target_y) = {
            let ant = &self.ants[idx];
            if let (Some(tx), Some(ty)) = (ant.target_x, ant.target_y) {
                (ant.x, ant.y, tx, ty)
            } else {
                return;
            }
        };

        // Calculate old distance for stuck detection
        let old_dist = ((target_x as i32 - ant_x as i32).abs()
            + (target_y as i32 - ant_y as i32).abs()) as f32;

        // Try cardinal directions first (more predictable)
        let directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]; // up, down, left, right

        for &dir in &directions {
            let new_x = ant_x as i32 + dir.0;
            let new_y = ant_y as i32 + dir.1;

            if new_x >= 0 && new_x < self.width as i32 && new_y >= 0 && new_y < self.height as i32 {
                let new_x = new_x as usize;
                let new_y = new_y as usize;

                if self.can_move_to(new_x, new_y) {
                    // Check if this direction gets us closer to target
                    let new_dist = ((target_x as i32 - new_x as i32).abs()
                        + (target_y as i32 - new_y as i32).abs())
                        as f32;

                    if new_dist <= old_dist {
                        self.ants[idx].x = new_x;
                        self.ants[idx].y = new_y;
                        // Reset stuck counter on progress
                        self.ants[idx].stuck_ticks = 0;
                        return;
                    }
                }
            }
        }

        // If no good cardinal direction, try any available tunnel
        for &dir in &directions {
            let new_x = ant_x as i32 + dir.0;
            let new_y = ant_y as i32 + dir.1;

            if new_x >= 0 && new_x < self.width as i32 && new_y >= 0 && new_y < self.height as i32 {
                let new_x = new_x as usize;
                let new_y = new_y as usize;

                if self.can_move_to(new_x, new_y) {
                    let new_dist = ((target_x as i32 - new_x as i32).abs()
                        + (target_y as i32 - new_y as i32).abs())
                        as f32;

                    self.ants[idx].x = new_x;
                    self.ants[idx].y = new_y;

                    // Check if we made progress or are stuck
                    if new_dist >= old_dist {
                        self.ants[idx].stuck_ticks += 1;
                        // If stuck for 3+ turns, switch to wandering
                        if self.ants[idx].stuck_ticks >= 3 {
                            self.ants[idx].state = AntState::Wandering;
                            self.ants[idx].target_x = None;
                            self.ants[idx].target_y = None;
                            self.ants[idx].stuck_ticks = 0;
                        }
                    } else {
                        self.ants[idx].stuck_ticks = 0;
                    }
                    return;
                }
            }
        }

        // If completely stuck (no moves available), increment stuck counter
        self.ants[idx].stuck_ticks += 1;
        if self.ants[idx].stuck_ticks >= 3 {
            self.ants[idx].state = AntState::Wandering;
            self.ants[idx].target_x = None;
            self.ants[idx].target_y = None;
            self.ants[idx].stuck_ticks = 0;
        }
    }

    fn try_dig(&mut self, idx: usize) -> bool {
        let mut rng = rand::rng();
        let ant = &self.ants[idx];

        // Calculate direction away from queen to encourage outward digging
        let dx_from_queen = ant.x as i32 - self.queen_pos.0 as i32;
        let dy_from_queen = ant.y as i32 - self.queen_pos.1 as i32;

        // Create directions biased away from queen
        let mut outward_directions = Vec::new();
        let mut other_directions = Vec::new();

        // Prefer cardinal directions for more tunnel-like structures
        let cardinal_directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1), // up, down, left, right
        ];
        let diagonal_directions = [
            (-1, -1),
            (1, -1),
            (-1, 1),
            (1, 1), // diagonals
        ];

        // Process cardinal directions first (weighted 3x) then diagonals for straighter tunnels
        let mut directions = Vec::new();

        // If ant has a last dig direction, prioritize continuing in that direction
        if let (Some(dx), Some(dy)) = (ant.last_dig_dx, ant.last_dig_dy) {
            let last_dir = (dx, dy);
            // Add the last direction 5 times to heavily bias continuation
            for _ in 0..5 {
                directions.push(last_dir);
            }
        }

        // Add cardinal directions 3 times to heavily weight them
        for _ in 0..3 {
            directions.extend_from_slice(&cardinal_directions);
        }
        // Then add diagonal directions once
        directions.extend_from_slice(&diagonal_directions);

        for &dir in &directions {
            let new_x = ant.x as i32 + dir.0;
            let new_y = ant.y as i32 + dir.1;

            if new_x >= 0 && new_x < self.width as i32 && new_y >= 0 && new_y < self.height as i32 {
                let new_x = new_x as usize;
                let new_y = new_y as usize;
                if self.grid[new_y][new_x] == CellType::Solid {
                    // Check if this direction moves away from queen
                    let new_dx_from_queen = new_x as i32 - self.queen_pos.0 as i32;
                    let new_dy_from_queen = new_y as i32 - self.queen_pos.1 as i32;
                    let old_distance = dx_from_queen.abs() + dy_from_queen.abs();
                    let new_distance = new_dx_from_queen.abs() + new_dy_from_queen.abs();

                    if new_distance > old_distance {
                        outward_directions.push(dir);
                    } else {
                        other_directions.push(dir);
                    }
                }
            }
        }

        // Try outward directions first (away from queen)
        if let Some(&dir) = outward_directions.first() {
            let new_x = ant.x as i32 + dir.0;
            let new_y = ant.y as i32 + dir.1;
            let new_x = new_x as usize;
            let new_y = new_y as usize;
            self.grid[new_y][new_x] = CellType::Tunnel;
            // Always move into outward tunnels to encourage sustained digging
            self.ants[idx].x = new_x;
            self.ants[idx].y = new_y;
            // Record the digging direction for tunnel continuation
            self.ants[idx].last_dig_dx = Some(dir.0);
            self.ants[idx].last_dig_dy = Some(dir.1);
            return true;
        }

        // If no outward directions, try other directions
        if let Some(&dir) = other_directions.first() {
            let new_x = ant.x as i32 + dir.0;
            let new_y = ant.y as i32 + dir.1;
            let new_x = new_x as usize;
            let new_y = new_y as usize;
            self.grid[new_y][new_x] = CellType::Tunnel;
            // Move into other tunnels 80% of the time to encourage tunnel formation
            if rng.random_bool(0.8) {
                self.ants[idx].x = new_x;
                self.ants[idx].y = new_y;
                // Record the digging direction for tunnel continuation
                self.ants[idx].last_dig_dx = Some(dir.0);
                self.ants[idx].last_dig_dy = Some(dir.1);
            }
            return true;
        }

        false
    }
}

#[derive(Debug)]
enum AntAction {
    ChangeState(AntState),
    MoveRandom,
    MoveRandomOrDig,
    MoveToTarget,
    TryDig,
}

fn render_world(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    world: &World,
) -> io::Result<()> {
    terminal.draw(|f| {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(0), Constraint::Length(9)])
            .split(f.area());

        // Calculate viewport to fit the world
        let world_area = chunks[0];
        let view_width = world_area.width.min(world.width as u16);
        let view_height = world_area.height.min(world.height as u16);

        // Center the view on the queen
        let offset_x = world.queen_pos.0.saturating_sub(view_width as usize / 2);
        let offset_y = world.queen_pos.1.saturating_sub(view_height as usize / 2);

        // Build the world view
        let mut lines = Vec::new();
        for y in offset_y..(offset_y + view_height as usize).min(world.height) {
            let mut spans = Vec::new();
            for x in offset_x..(offset_x + view_width as usize).min(world.width) {
                // Check for ant
                let mut has_ant = false;
                let mut ant_state = AntState::Wandering;
                for ant in &world.ants {
                    if ant.x == x && ant.y == y {
                        has_ant = true;
                        ant_state = ant.state;
                        break;
                    }
                }

                if has_ant {
                    let color = match ant_state {
                        AntState::Carrying => Color::Yellow,
                        AntState::Digging => Color::Red,
                        AntState::Foraging => Color::Green,
                        AntState::Wandering => Color::White,
                    };
                    spans.push(Span::styled("●", Style::default().fg(color)));
                } else {
                    let (ch, color) = match world.grid[y][x] {
                        CellType::Solid => ('░', Color::DarkGray),
                        CellType::Tunnel => {
                            // Check for distress pheromones in tunnels
                            if let Some(pheromone) = world.pheromones.get(&(x, y)) {
                                if pheromone.distress > 1.0 {
                                    ('!', Color::Red) // Show distress signals
                                } else {
                                    (' ', Color::Black)
                                }
                            } else {
                                (' ', Color::Black)
                            }
                        }
                        CellType::Food => ('*', Color::Magenta),
                        CellType::Queen => ('Q', Color::Cyan),
                    };
                    spans.push(Span::styled(ch.to_string(), Style::default().fg(color)));
                }
            }
            lines.push(Line::from(spans));
        }

        let world_widget = Paragraph::new(lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Ant Colony ")
                .style(Style::default().fg(Color::White)),
        );
        f.render_widget(world_widget, chunks[0]);

        // Stats panel
        let stats = vec![
            Line::from(vec![
                Span::styled("Colony Age: ", Style::default().fg(Color::Gray)),
                Span::styled(
                    format!("{} ticks", world.ticks),
                    Style::default().fg(Color::White),
                ),
            ]),
            Line::from(vec![
                Span::styled("Population: ", Style::default().fg(Color::Gray)),
                Span::styled(
                    format!("{} ants", world.ants.len()),
                    Style::default().fg(Color::Green),
                ),
            ]),
            Line::from(vec![
                Span::styled("Food Collected: ", Style::default().fg(Color::Gray)),
                Span::styled(
                    format!("{}", world.food_collected),
                    Style::default().fg(Color::Magenta),
                ),
            ]),
            Line::from(vec![
                Span::styled("Health: ", Style::default().fg(Color::Gray)),
                Span::styled(
                    format!(
                        "{}{}",
                        "█".repeat((world.health / 5.0) as usize),
                        "░".repeat(20 - (world.health / 5.0) as usize)
                    ),
                    Style::default().fg(if world.health > 70.0 {
                        Color::Green
                    } else if world.health > 40.0 {
                        Color::Yellow
                    } else {
                        Color::Red
                    }),
                ),
                Span::styled(
                    format!(" {:.0}%", world.health),
                    Style::default().fg(Color::White),
                ),
            ]),
            Line::from(vec![
                Span::styled("Distress Signals: ", Style::default().fg(Color::Gray)),
                Span::styled(
                    format!(
                        "{}",
                        world
                            .pheromones
                            .values()
                            .filter(|p| p.distress > 1.0)
                            .count()
                    ),
                    Style::default().fg(Color::Red),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled(
                    "q",
                    Style::default()
                        .fg(Color::Yellow)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(" quit & save  ", Style::default().fg(Color::Gray)),
                Span::styled(
                    "r",
                    Style::default()
                        .fg(Color::Yellow)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(" reset", Style::default().fg(Color::Gray)),
            ]),
        ];
        let stats_widget = Paragraph::new(stats).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Stats ")
                .style(Style::default().fg(Color::White)),
        );
        f.render_widget(stats_widget, chunks[1]);
    })?;
    Ok(())
}

fn main() -> io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let reset = args.contains(&"--reset".to_string());

    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    terminal.clear()?;

    let size = terminal.size()?;
    let width = (size.width as usize).clamp(60, 100);
    let height = (size.height as usize - 11).clamp(20, 40);

    let mut world = if reset {
        println!("Starting fresh colony!");
        World::new(width, height)
    } else {
        match World::load() {
            Ok(w) => {
                println!("Loaded saved colony!");
                w
            }
            Err(_) => {
                println!("Starting fresh colony!");
                World::new(width, height)
            }
        }
    };

    let mut last_tick = Instant::now();
    let mut current_width = width;
    let mut current_height = height;
    let mut game_over = false;

    loop {
        // Check for terminal resize
        let new_size = terminal.size()?;
        let new_width = (new_size.width as usize).clamp(60, 100);
        let new_height = (new_size.height as usize - 11).clamp(20, 40);

        if new_width != current_width || new_height != current_height {
            // Terminal size changed - adjust world dimensions
            current_width = new_width;
            current_height = new_height;
            // Note: This keeps the existing world state but adjusts viewport
            // For a full re-init, uncomment the line below:
            // world = World::new(current_width, current_height);
        }

        if game_over {
            // Game over screen - don't update world or render normally
            terminal.draw(|f| {
                f.render_widget(
                    Paragraph::new("Colony collapsed! Press r to restart or q to quit.")
                        .block(Block::default().borders(Borders::ALL).title("Game Over")),
                    f.area(),
                );
            })?;

            // Wait for key input
            if event::poll(Duration::from_secs(1))?
                && let Event::Key(key) = event::read()?
            {
                match key.code {
                    KeyCode::Char('r') => {
                        world = World::new(current_width, current_height);
                        game_over = false; // Resume normal game
                    }
                    KeyCode::Char('q') => {
                        world.save()?;
                        break;
                    }
                    _ => {}
                }
            }
        } else {
            // Normal game loop
            render_world(&mut terminal, &world)?;

            let timeout = TICK_RATE.saturating_sub(last_tick.elapsed());
            if event::poll(timeout)?
                && let Event::Key(key) = event::read()?
            {
                match key.code {
                    KeyCode::Char('q') => {
                        world.save()?;
                        break;
                    }
                    KeyCode::Char('r') => {
                        world = World::new(current_width, current_height);
                    }
                    _ => {}
                }
            }

            if last_tick.elapsed() >= TICK_RATE {
                world.update();
                last_tick = Instant::now();

                // Check for game over condition
                if world.health <= 0.0 {
                    game_over = true;
                }
            }
        }
    }

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;

    Ok(())
}
