import random
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Parameters
M = 4  # Number of males
N = 4  # Number of females
iterations = 500  # Number of random encounters

# Initialize likeness scores (initially set to 0 for all pairs)
likeness_scores = {(m, f): 0 for m in range(1, M+1) for f in range(1, N+1)}
friendships = {(m, f): False for m in range(1, M+1) for f in range(1, N+1)}
relationships = {(m, f): False for m in range(1, M+1) for f in range(1, N+1)}

# Simulation
for _ in range(iterations):
    male = random.randint(1, M)
    female = random.randint(1, N)
    change = random.choice([-1, 0, 1])

    # Check if the male or female is already in a relationship
    male_in_relationship = any(relationships[(male, f)] for f in range(1, N+1))
    female_in_relationship = any(relationships[(m, female)] for m in range(1, M+1))

    # If either is in a relationship, skip this encounter
    if male_in_relationship or female_in_relationship:
        continue

    likeness_scores[(male, female)] += change

    # Update status
    if likeness_scores[(male, female)] > 4:
        friendships[(male, female)] = True
    else:
        friendships[(male, female)] = False

    if likeness_scores[(male, female)] > 7:
        relationships[(male, female)] = True
    else:
        relationships[(male, female)] = False

# Prepare data for visualization
likeness_matrix = pd.DataFrame(0, index=range(1, M+1), columns=range(1, N+1))
friendship_matrix = pd.DataFrame(False, index=range(1, M+1), columns=range(1, N+1))
relationship_matrix = pd.DataFrame(False, index=range(1, M+1), columns=range(1, N+1))

for (male, female), score in likeness_scores.items():
    likeness_matrix.at[male, female] = score
    friendship_matrix.at[male, female] = friendships[(male, female)]
    relationship_matrix.at[male, female] = friendships[(male, female)]

# Visualize likeness scores
plt.figure(figsize=(18, 6))

plt.subplot(1, 3, 1)
sns.heatmap(likeness_matrix, annot=True, cmap='coolwarm', cbar=True, linewidths=.5)
plt.title('Likeness Scores')
plt.xlabel('Female')
plt.ylabel('Male')

# Visualize friendships
plt.subplot(1, 3, 2)
sns.heatmap(friendship_matrix, annot=True, cmap='Greens', cbar=False, linewidths=.5)
plt.title('Friendships')
plt.xlabel('Female')
plt.ylabel('Male')

# Visualize relationships
plt.subplot(1, 3, 3)
sns.heatmap(relationship_matrix, annot=True, cmap='Reds', cbar=False, linewidths=.5)
plt.title('Relationships')
plt.xlabel('Female')
plt.ylabel('Male')

plt.tight_layout()
plt.show()

