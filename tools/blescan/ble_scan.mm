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

static NSString *uuidList(NSArray<CBUUID *> *uuids) {
    if (!uuids || uuids.count == 0) return @"-";
    NSMutableArray<NSString *> *out = [NSMutableArray array];
    for (CBUUID *u in uuids) [out addObject:u.UUIDString];
    return [out componentsJoinedByString:@", "];
}

@interface Scanner : NSObject <CBCentralManagerDelegate>
@property(nonatomic, strong) CBCentralManager *central;
@property(nonatomic, strong) NSString *nameFilter;
@end

@implementation Scanner

- (instancetype)initWithFilter:(NSString *)filter {
    self = [super init];
    if (self) {
        _nameFilter = filter;
        _central = [[CBCentralManager alloc] initWithDelegate:self
                                                        queue:dispatch_get_main_queue()
                                                      options:nil];
    }
    return self;
}

- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    const char *state = "unknown";
    switch (central.state) {
        case CBManagerStatePoweredOn:    state = "poweredOn"; break;
        case CBManagerStatePoweredOff:   state = "poweredOff"; break;
        case CBManagerStateUnauthorized: state = "unauthorized"; break;
        case CBManagerStateUnsupported:  state = "unsupported"; break;
        case CBManagerStateResetting:    state = "resetting"; break;
        default: break;
    }

    fprintf(stderr, "Bluetooth state: %s\n", state);

    if (central.state == CBManagerStatePoweredOn) {
        NSDictionary *opts = @{
            CBCentralManagerScanOptionAllowDuplicatesKey: @YES
        };

        // nil services => discover every advertising BLE peripheral.
        [central scanForPeripheralsWithServices:nil options:opts];
        fprintf(stderr, "Scanning all BLE advertisements (duplicates enabled)...\n");
        if (self.nameFilter.length)
            fprintf(stderr, "Display filter: name contains \"%s\"\n",
                    self.nameFilter.UTF8String);
    }
}

- (void)centralManager:(CBCentralManager *)central
 didDiscoverPeripheral:(CBPeripheral *)peripheral
     advertisementData:(NSDictionary<NSString *, id> *)adv
                  RSSI:(NSNumber *)RSSI {

    NSString *localName = adv[CBAdvertisementDataLocalNameKey];
    NSString *name = localName ?: peripheral.name ?: @"<no name>";

    NSData *manufacturer = adv[CBAdvertisementDataManufacturerDataKey];

    // Don't filter a nameless packet if it carries Sensirion company bytes.
    BOOL looksSensirion = NO;
    if (manufacturer.length >= 2) {
        const uint8_t *p =
            static_cast<const uint8_t *>(manufacturer.bytes);
        looksSensirion =
            (p[0] == 0x06 && p[1] == 0xD5) ||
            (p[0] == 0xD5 && p[1] == 0x06);
    }

    if (self.nameFilter.length &&
        [name rangeOfString:self.nameFilter
                    options:NSCaseInsensitiveSearch].location == NSNotFound &&
        !looksSensirion) {
        return;
    }

    NSArray<CBUUID *> *services =
        adv[CBAdvertisementDataServiceUUIDsKey];

    NSDictionary<CBUUID *, NSData *> *serviceData =
        adv[CBAdvertisementDataServiceDataKey];

    NSNumber *connectable =
        adv[CBAdvertisementDataIsConnectable];

    NSNumber *txPower =
        adv[CBAdvertisementDataTxPowerLevelKey];

    printf("\n=== BLE advertisement ===\n");
    printf("time:          %s\n",
           [[[NSDate date] descriptionWithLocale:
              [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"]]
             UTF8String]);
    printf("name:          %s\n", name.UTF8String);
    printf("peripheral id: %s\n",
           peripheral.identifier.UUIDString.UTF8String);
    printf("RSSI:          %s dBm\n", RSSI.stringValue.UTF8String);
    printf("connectable:   %s\n",
           connectable ? (connectable.boolValue ? "yes" : "no") : "?");
    printf("tx power:      %s\n",
           txPower ? txPower.stringValue.UTF8String : "-");
    printf("manufacturer:  %s\n",
           hexString(manufacturer).UTF8String);
    printf("service UUIDs: %s\n",
           uuidList(services).UTF8String);

    if (serviceData.count) {
        printf("service data:\n");
        for (CBUUID *uuid in serviceData) {
            printf("  %s: %s\n",
                   uuid.UUIDString.UTF8String,
                   hexString(serviceData[uuid]).UTF8String);
        }
    } else {
        printf("service data:  -\n");
    }

    // Also dump every CoreBluetooth advertisement dictionary key/value.
    printf("raw CoreBluetooth dictionary:\n");
    for (NSString *key in adv) {
        id value = adv[key];
        if ([value isKindOfClass:[NSData class]]) {
            printf("  %s = <%s>\n",
                   key.UTF8String,
                   hexString((NSData *)value).UTF8String);
        } else {
            printf("  %s = %s\n",
                   key.UTF8String,
                   [[value description] UTF8String]);
        }
    }
    fflush(stdout);
}

@end

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        NSString *filter = @"Unni-CO2";

        if (argc >= 2) {
            if (strcmp(argv[1], "--all") == 0)
                filter = @"";
            else
                filter = [NSString stringWithUTF8String:argv[1]];
        }

        fprintf(stderr,
                "BLE scanner. Ctrl-C to stop.\n"
                "Use --all to print every discovered BLE peripheral.\n");

        Scanner *scanner = [[Scanner alloc] initWithFilter:filter];
        (void)scanner;

        [[NSRunLoop mainRunLoop] run];
    }
    return 0;
}
