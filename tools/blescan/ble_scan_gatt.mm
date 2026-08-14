// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>

static NSString *hexString(NSData *data) {
    if (!data) return @"-";
    const unsigned char *bytes =
        static_cast<const unsigned char *>(data.bytes);
    NSMutableString *s = [NSMutableString stringWithCapacity:data.length * 3];
    for (NSUInteger i = 0; i < data.length; i++) {
        if (i) [s appendString:@" "];
        [s appendFormat:@"%02X", bytes[i]];
    }
    return s;
}

@interface Scanner : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>
@property(nonatomic, strong) CBCentralManager *central;
@property(nonatomic, strong) CBPeripheral *target;
@property(nonatomic, assign) BOOL connecting;
@end

@implementation Scanner

- (instancetype)init {
    self = [super init];
    if (self) {
        _central = [[CBCentralManager alloc] initWithDelegate:self
                                                        queue:dispatch_get_main_queue()
                                                      options:nil];
    }
    return self;
}

- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    fprintf(stderr, "Bluetooth state: %ld\n", (long)central.state);
    if (central.state == CBManagerStatePoweredOn) {
        NSDictionary *opts = @{
            CBCentralManagerScanOptionAllowDuplicatesKey: @YES
        };
        [central scanForPeripheralsWithServices:nil options:opts];
        fprintf(stderr, "Scanning for Unni-CO2...\n");
    }
}

- (void)centralManager:(CBCentralManager *)central
 didDiscoverPeripheral:(CBPeripheral *)peripheral
     advertisementData:(NSDictionary<NSString *, id> *)adv
                  RSSI:(NSNumber *)RSSI {

    NSString *name = adv[CBAdvertisementDataLocalNameKey] ?: peripheral.name ?: @"";
    NSData *manufacturer = adv[CBAdvertisementDataManufacturerDataKey];

    BOOL nameMatch =
        [name rangeOfString:@"Unni-CO2"
                    options:NSCaseInsensitiveSearch].location != NSNotFound;

    BOOL sensirion = NO;
    if (manufacturer.length >= 2) {
        const uint8_t *p =
            static_cast<const uint8_t *>(manufacturer.bytes);
        sensirion = (p[0] == 0x06 && p[1] == 0xD5) ||
                    (p[0] == 0xD5 && p[1] == 0x06);
    }

    if (!nameMatch && !sensirion)
        return;

    printf("\nFOUND %s RSSI=%s manufacturer=<%s>\n",
           name.UTF8String,
           RSSI.stringValue.UTF8String,
           hexString(manufacturer).UTF8String);

    if (!self.connecting && !self.target) {
        self.connecting = YES;
        self.target = peripheral;
        self.target.delegate = self;
        [central stopScan];
        fprintf(stderr, "Connecting...\n");
        [central connectPeripheral:peripheral options:nil];
    }
}

- (void)centralManager:(CBCentralManager *)central
 didConnectPeripheral:(CBPeripheral *)peripheral {
    fprintf(stderr, "CONNECTED: %s\n", peripheral.identifier.UUIDString.UTF8String);
    [peripheral discoverServices:nil];
}

- (void)centralManager:(CBCentralManager *)central
 didFailToConnectPeripheral:(CBPeripheral *)peripheral
                 error:(NSError *)error {
    fprintf(stderr, "CONNECT FAILED: %s\n",
            error ? error.localizedDescription.UTF8String : "unknown");
    self.connecting = NO;
    self.target = nil;
}

- (void)centralManager:(CBCentralManager *)central
 didDisconnectPeripheral:(CBPeripheral *)peripheral
                 error:(NSError *)error {
    fprintf(stderr, "DISCONNECTED%s%s\n",
            error ? ": " : "",
            error ? error.localizedDescription.UTF8String : "");
}

- (void)peripheral:(CBPeripheral *)peripheral
 didDiscoverServices:(NSError *)error {
    if (error) {
        fprintf(stderr, "Service discovery error: %s\n",
                error.localizedDescription.UTF8String);
        return;
    }

    printf("\n=== GATT services ===\n");
    for (CBService *service in peripheral.services) {
        printf("SERVICE %s%s\n",
               service.UUID.UUIDString.UTF8String,
               service.isPrimary ? " primary" : "");
        [peripheral discoverCharacteristics:nil forService:service];
    }
    fflush(stdout);
}

- (void)peripheral:(CBPeripheral *)peripheral
 didDiscoverCharacteristicsForService:(CBService *)service
 error:(NSError *)error {
    if (error) {
        fprintf(stderr, "Characteristic discovery error for %s: %s\n",
                service.UUID.UUIDString.UTF8String,
                error.localizedDescription.UTF8String);
        return;
    }

    for (CBCharacteristic *ch in service.characteristics) {
        printf("  CHAR %s props=0x%lx\n",
               ch.UUID.UUIDString.UTF8String,
               (unsigned long)ch.properties);

        if (ch.properties & CBCharacteristicPropertyRead)
            [peripheral readValueForCharacteristic:ch];

        [peripheral discoverDescriptorsForCharacteristic:ch];
    }
    fflush(stdout);
}

- (void)peripheral:(CBPeripheral *)peripheral
 didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic
 error:(NSError *)error {
    if (error) {
        printf("    READ %s error=%s\n",
               characteristic.UUID.UUIDString.UTF8String,
               error.localizedDescription.UTF8String);
    } else {
        printf("    READ %s = <%s>\n",
               characteristic.UUID.UUIDString.UTF8String,
               hexString(characteristic.value).UTF8String);
    }
    fflush(stdout);
}

- (void)peripheral:(CBPeripheral *)peripheral
 didDiscoverDescriptorsForCharacteristic:(CBCharacteristic *)characteristic
 error:(NSError *)error {
    if (error) return;

    for (CBDescriptor *d in characteristic.descriptors) {
        printf("    DESC %s\n", d.UUID.UUIDString.UTF8String);
    }
    fflush(stdout);
}

@end

int main(void) {
    @autoreleasepool {
        fprintf(stderr, "BLE GATT scanner. Ctrl-C to stop.\n");
        Scanner *scanner = [[Scanner alloc] init];
        (void)scanner;
        [[NSRunLoop mainRunLoop] run];
    }
    return 0;
}
