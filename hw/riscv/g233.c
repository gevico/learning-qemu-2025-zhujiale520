/*
 * QEMU RISC-V G233 Board (Learning QEMU 2025)
 *
 * Copyright (c) 2025 Zevorn(Chao Liu) chao.liu@yeah.net
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "system/system.h"
#include "system/memory.h"
#include "target/riscv/cpu.h"
#include "chardev/char.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/riscv/g233.h"
#include "hw/riscv/boot.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/misc/unimp.h"
#include "hw/char/pl011.h"
#include "hw/gpio/sifive_gpio.h" /* 用于 TYPE_SIFIVE_GPIO */

/* TODO: you need include some header files */

static const MemMapEntry g233_memmap[] = {
    [G233_DEV_MROM] =     {     0x1000,     0x2000 },
    [G233_DEV_CLINT] =    {  0x2000000,    0x10000 },
    [G233_DEV_PLIC] =     {  0xc000000,  0x4000000 },
    [G233_DEV_UART0] =    { 0x10000000,     0x1000 },
    [G233_DEV_GPIO0] =    { 0x10012000,     0x1000 },
    [G233_DEV_PWM0] =     { 0x10015000,     0x1000 },
    [G233_DEV_DRAM] =     { 0x80000000, 0x40000000 },
};

/* 初始化 SoC：创建子设备（hart array、gpio 等）并设置属性 */
static void g233_soc_init(Object *obj)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    G233SoCState *s = RISCV_G233_SOC(obj);

    /* 创建 hart 数组并设置属性
     * 注意：resetvec 在 mask ROM 的 0x1004 处，num-harts 使用 machine 的 smp.cpus */
    object_initialize_child(obj, "cpus", &s->cpus, TYPE_RISCV_HART_ARRAY);
    object_property_set_int(OBJECT(&s->cpus), "num-harts", ms->smp.cpus,
                            &error_abort);
    object_property_set_int(OBJECT(&s->cpus), "resetvec", 0x1004,
                            &error_abort);

    /* GPIO（作为 child） */
    object_initialize_child(obj, "riscv.g233.gpio0", &s->gpio,
                            TYPE_SIFIVE_GPIO);

    /* uart/pwm 等可以在 realize 时创建或由 create_xxx 函数创建，
     * pwm 在这里使用未实现设备占位（create_unimplemented_device）。 */
}

/* realize：在此处把设备真实化（create/realize/qdev_realize），并把 MMIO/IRQ 连接好 */
static void g233_soc_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    G233SoCState *s = RISCV_G233_SOC(dev);
    MemoryRegion *sys_mem = get_system_memory();
    const MemMapEntry *memmap = g233_memmap;

    /* CPUs realize
     * 先确保 hart array 使用 machine 的 cpu_type，这样在 realize 时会创建 RISCVCPU 对象
     * 必须在配置 PLIC/CLINT 并连接外设中断之前创建 CPU，避免中断连接出问题 */
    object_property_set_str(OBJECT(&s->cpus), "cpu-type", ms->cpu_type,
                            &error_abort);
    if (!qdev_realize(DEVICE(&s->cpus), NULL, errp)) {
        return;
    }

    /* Mask ROM（ROM 区） */
    memory_region_init_rom(&s->mask_rom, OBJECT(dev), "riscv.g233.mrom",
                           memmap[G233_DEV_MROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[G233_DEV_MROM].base,
                                &s->mask_rom);

    /* MMIO 中断控制器：PLIC / CLINT */
    s->plic = sifive_plic_create(memmap[G233_DEV_PLIC].base,
                                 (char *)G233_PLIC_HART_CONFIG, ms->smp.cpus, 0,
                                 G233_PLIC_NUM_SOURCES,
                                 G233_PLIC_NUM_PRIORITIES,
                                 G233_PLIC_PRIORITY_BASE,
                                 G233_PLIC_PENDING_BASE,
                                 G233_PLIC_ENABLE_BASE,
                                 G233_PLIC_ENABLE_STRIDE,
                                 G233_PLIC_CONTEXT_BASE,
                                 G233_PLIC_CONTEXT_STRIDE,
                                 memmap[G233_DEV_PLIC].size);
    riscv_aclint_swi_create(memmap[G233_DEV_CLINT].base,
                            0, ms->smp.cpus, false);
    riscv_aclint_mtimer_create(memmap[G233_DEV_CLINT].base +
                               RISCV_ACLINT_SWI_SIZE,
                               RISCV_ACLINT_DEFAULT_MTIMER_SIZE,
                               0, ms->smp.cpus,
                               RISCV_ACLINT_DEFAULT_MTIMECMP,
                               RISCV_ACLINT_DEFAULT_MTIME,
                               32768, false); /* TODO: set default freq */

    /* GPIO realize 并映射寄存器 */
    if (!qdev_realize(DEVICE(&s->gpio), NULL, errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, memmap[G233_DEV_GPIO0].base);

    /* 将 GPIO 的 gpio lines 传给 SOC，使 board 层可访问 */
    qdev_pass_gpios(DEVICE(&s->gpio), dev, NULL);

    /* 将 GPIO 的每个引脚中断接到 PLIC */
    for (int i = 0; i < 32; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), i,
                           qdev_get_gpio_in(DEVICE(s->plic),
                                            G233_GPIO0_IRQ0 + i));
    }

    /* 添加 UART（PL011）
     * pl011_create 会创建并注册 UART 设备并返回 DeviceState *
     * 同时把 UART IRQ 与 PLIC 链接（pl011_create 内部会处理中断线，但这里仍保持 PLIC 的 gpio 查询） */
    s->uart0 = pl011_create(memmap[G233_DEV_UART0].base,
                            qdev_get_gpio_in(DEVICE(s->plic), G233_UART0_IRQ),
                            serial_hd(0));

    /* SiFive.PWM0：暂时使用未实现占位 */
    create_unimplemented_device("riscv.g233.pwm0",
        memmap[G233_DEV_PWM0].base, memmap[G233_DEV_PWM0].size);
}

static void g233_soc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = g233_soc_realize;
}

static const TypeInfo g233_soc_type_info = {
    .name = TYPE_RISCV_G233_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(G233SoCState),
    .instance_init = g233_soc_init,
    .class_init = g233_soc_class_init,
};

static void g233_soc_register_types(void)
{
    type_register_static(&g233_soc_type_info);
}

type_init(g233_soc_register_types)

static void g233_machine_init(MachineState *machine)
{
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    const MemMapEntry *memmap = g233_memmap;

    G233MachineState *s = RISCV_G233_MACHINE(machine);
    int i;
    RISCVBootInfo boot_info;

    if (machine->ram_size < mc->default_ram_size) {
        char *sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be %s", sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    /* Initialize SoC: 把 soc 作为 machine 的 child 创建并 realize */
    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_RISCV_G233_SOC);
    qdev_realize(DEVICE(&s->soc), NULL, &error_fatal);

    /* Data Memory (DDR RAM) 映射到系统地址空间的 DRAM 基址 */
    memory_region_add_subregion(get_system_memory(),
                                memmap[G233_DEV_DRAM].base,
                                machine->ram);

    /* Mask ROM reset vector */
    uint32_t reset_vec[5];
    reset_vec[1] = 0x0010029b; /* 0x1004: addiw  t0, zero, 1 */
    reset_vec[2] = 0x01f29293; /* 0x1008: slli   t0, t0, 0x1f */
    reset_vec[3] = 0x00028067; /* 0x100c: jr     t0 */
    reset_vec[0] = reset_vec[4] = 0;

    /* copy in the reset vector in little_endian byte order */
    for (i = 0; i < sizeof(reset_vec) >> 2; i++) {
        reset_vec[i] = cpu_to_le32(reset_vec[i]);
    }
    rom_add_blob_fixed_as("mrom.reset", reset_vec, sizeof(reset_vec),
                          memmap[G233_DEV_MROM].base, &address_space_memory);

    /* 初始化 riscv 的引导信息（会使用 s->soc.cpus 内的信息） */
    riscv_boot_info_init(&boot_info, &s->soc.cpus);
    if (machine->kernel_filename) {
        riscv_load_kernel(machine, &boot_info,
                          memmap[G233_DEV_DRAM].base,
                          false, NULL);
    }
}

static void g233_machine_instance_init(Object *obj)
{
}

static void g233_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "QEMU RISC-V G233 Board with Learning QEMU 2025";
    mc->init = g233_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = TYPE_RISCV_CPU_GEVICO_G233;
    mc->default_ram_id = "riscv.g233.ram"; /* DDR */
    mc->default_ram_size = g233_memmap[G233_DEV_DRAM].size;
}

static const TypeInfo g233_machine_typeinfo = {
    .name       = TYPE_RISCV_G233_MACHINE,
    .parent     = TYPE_MACHINE,
    .class_init = g233_machine_class_init,
    .instance_init = g233_machine_instance_init,
    .instance_size = sizeof(G233MachineState),
};

static void g233_machine_init_register_types(void)
{
    type_register_static(&g233_machine_typeinfo);
}

type_init(g233_machine_init_register_types)
