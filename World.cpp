/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#include "World.h"
#include "Body.h"
#include "Joint.h"

using std::vector;
using std::map;
using std::pair;

typedef map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;

bool World::accumulateImpulses = true; // 这是干嘛的?
bool World::warmStarting = true; // 热启动, 缓存计算中间成果, 值得一看
bool World::positionCorrection = true; // 位置校准, 需要格外留意, 尤其是配套的抗抖动方案

void World::Add(Body* body)
{
	bodies.push_back(body);
}

void World::Add(Joint* joint)
{
	joints.push_back(joint);
}

void World::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}


// 遍历所有物体, 把发生接触了的, 挑出来
void World::BroadPhase()
{
	// O(n^2) broad-phase
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* bi = bodies[i];

		for (int j = i + 1; j < (int)bodies.size(); ++j)
		{
			Body* bj = bodies[j];

			// 两个非动力学物体之间不发生碰撞
			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			Arbiter newArb(bi, bj);
			ArbiterKey key(bi, bj); // 显然是先做个排序然后生成key, 来避免两个物体顺序的影响

			if (newArb.numContacts > 0) // 挑出来碰撞了的物体
			{
				// 去掉重复的pair
				ArbIter iter = arbiters.find(key); // Map.find(key), 找不到就返回尾部迭代器
				if (iter == arbiters.end())
				{
					arbiters.insert(ArbPair(key, newArb));
				}
				else
				{
					// first是key, second是value
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
			else
			{
				arbiters.erase(key);
			}
		}
	}
}

void World::Step(float dt)
{
	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	// Determine overlapping(重叠) bodies and update contact points.
	BroadPhase();

	// Integrate(整合) forces. // 把力转换为速度
	// 生成力然后计算速度, 为嘛不用冲量?
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];

		if (b->invMass == 0.0f)
			continue;

		// inv 是倒数.....
		b->velocity += dt * (gravity + b->invMass * b->force);
		b->angularVelocity += dt * b->invI * b->torque; // 角速度
	}

	// Perform pre-steps.
	for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		// 看来每一个Arbiter都有一个PreStep, 应该是用来做位置校准和抗抖动的吧
		arb->second.PreStep(inv_dt);
	}

	for (int i = 0; i < (int)joints.size(); ++i)
	{
		// 原来每一个关节也有呀
		joints[i]->PreStep(inv_dt);
	}

	// Perform iterations
	for (int i = 0; i < iterations; ++i)
	{
		// 动力学的核心部分, 话说, 上面都已经生成过速度了耶
		// 看来作者让force和impulse使用在了不同的场景
		// 按照常理, 刚体碰撞应该是只能用impulse去解(因为碰撞力无限大, 碰撞时间无限小)
		// 力估计是用来处理堆叠稳定性的
		for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
		{
			arb->second.ApplyImpulse();
		}

		for (int j = 0; j < (int)joints.size(); ++j)
		{
			// 看来关节也只能用impulse
			// 如果用力的话就变成弹簧了
			// (弹性连接比关节,绳子,木棍等链接要容易很多)
			joints[j]->ApplyImpulse();
		}
	}

	// Integrate Velocities
	// 位移生效
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];

		b->position += dt * b->velocity;
		b->rotation += dt * b->angularVelocity;

		b->force.Set(0.0f, 0.0f);
		b->torque = 0.0f; // 扭矩
	}
}
